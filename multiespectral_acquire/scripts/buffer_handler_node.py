#!/usr/bin/env python3
"""
Generic buffer handler node in Python - truly type-agnostic
Buffers any ROS message and stores synchronized with master trigger
"""

import rospy
import yaml
import os
import struct
from collections import deque
from pathlib import Path
import importlib
import cv2
import numpy as np
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2

class TimedBuffer:
    """Generic buffer for timestamped data (thread-safe)"""
    def __init__(self, max_size=50):
        self.buffer = deque(maxlen=max_size)
        self.lock = threading.Lock()  # Protect buffer from concurrent access
        self.last_add_time = None  # Track when buffer was last updated
    
    def add(self, timestamp, data):
        with self.lock:
            self.buffer.append({'timestamp': timestamp, 'data': data})
            self.last_add_time = rospy.Time.now()
    
    def find_closest(self, target_timestamp, max_diff_sec=0.1):
        with self.lock:
            if not self.buffer:
                return None
            
            # Create a snapshot to avoid "deque mutated during iteration" error
            buffer_snapshot = list(self.buffer)
        
        # Find closest outside the lock (safe to iterate over list)
        closest = min(buffer_snapshot, key=lambda x: abs(x['timestamp'] - target_timestamp))
        time_diff = abs(closest['timestamp'] - target_timestamp) / 1e9
        
        if time_diff <= max_diff_sec:
            return closest
        return None
    
    def is_empty(self):
        with self.lock:
            return len(self.buffer) == 0
    
    def get_time_since_last_update(self):
        """Get seconds since last buffer update"""
        with self.lock:
            if self.last_add_time is None:
                return None
            return (rospy.Time.now() - self.last_add_time).to_sec()


class GenericBufferHandler:
    """Generic buffer handler for any ROS message type"""
    
    def __init__(self):
        rospy.init_node('buffer_handler_node', anonymous=True)
        
        # Parameters
        self.handler_type = rospy.get_param('~handler_type', 'generic')
        self.data_topic = rospy.get_param('~data_topic')
        self.master_topic = rospy.get_param('~master_topic', '')
        self.store_data = rospy.get_param('~store_data', True)
        
        # Auto-detect store_all: if no master_topic, store everything
        if not self.master_topic:
            self.store_all = True
            rospy.loginfo(f"[BufferHandler::{self.data_topic}] No master_topic → auto-enabling store_all mode")
        else:
            self.store_all = rospy.get_param('~store_all', False)
        
        # Max time diff - default based on typical sensor rates
        # If not specified: 0.1s default (good for 10Hz sensors)
        self.max_time_diff = rospy.get_param('~max_time_diff', 0.1)
        
        # Exposure time for calculating half_exposure (optional, for synced sensors)
        self.exposure_time_ns = rospy.get_param('~exposure_time_ns', 0)
        if self.exposure_time_ns > 0:
            rospy.loginfo(f"[BufferHandler::{self.data_topic}] Using FIXED exposure_time: {self.exposure_time_ns}ns ({self.exposure_time_ns/1e6:.2f}ms)")
        else:
            rospy.loginfo(f"[BufferHandler::{self.data_topic}] Will AUTO-READ exposure_time from message metadata (if available)")

        # Storage path
        self.storage_path = rospy.get_param('~output_path')
        
        # Don't create folder yet - wait until recording is enabled
        self.storage_path_created = False
        
        # Recording control flag (default: disabled until explicitly enabled)
        self.recording_enabled = False
        
        rospy.loginfo(f"[BufferHandler::{self.data_topic}] Storage path: {self.storage_path}")
        rospy.loginfo(f"[BufferHandler::{self.data_topic}] Handler type: {self.handler_type}")
        rospy.loginfo(f"[BufferHandler::{self.data_topic}] Mode: {'STORE ALL' if self.store_all else f'Master-triggered (max_diff={self.max_time_diff}s)'}")
        
        if self.store_data:
            rospy.loginfo(f"[BufferHandler::{self.data_topic}] Storage: ENABLED (waiting for /Multiespectral/recording_enabled)")
        else:
            rospy.loginfo(f"[BufferHandler::{self.data_topic}] Storage: DISABLED (sync-only mode)")
        
        # Buffer
        self.buffer = TimedBuffer()
        self.bridge = CvBridge()
        
        # Publisher for synchronized data (will be created dynamically with correct type)
        self.sync_topic = self.data_topic.rstrip('/') + '_sync'
        self.sync_pub = None  # Will be initialized on first message
        self.sync_msg_class = None
        rospy.loginfo(f"[BufferHandler::{self.data_topic}] Will republish to: {self.sync_topic}")
        
        # Subscribe to recording control topic (shared across all handlers)
        self.control_sub = rospy.Subscriber('/Multiespectral/recording_enabled', Bool, self.recording_control_callback)
        
        # Subscribe to data topic (generic subscriber)
        self.data_sub = rospy.Subscriber(self.data_topic, rospy.AnyMsg, self.data_callback)
        
        # Get resolved topic name (absolute path) from subscriber
        self.resolved_topic = self.data_sub.resolved_name
        rospy.loginfo(f"[BufferHandler::{self.data_topic}] Resolved to: {self.resolved_topic}")
        
        # Subscribe to master if needed (for sync mode only)
        if self.master_topic and not self.store_all:
            try:
                from multiespectral_acquire.msg import ImageWithMetadata
                self.master_sub = rospy.Subscriber(self.master_topic, ImageWithMetadata, self.master_callback)
            except ImportError:
                rospy.logerr(f"[BufferHandler::{self.data_topic}] Cannot import ImageWithMetadata")
        
        # Topic monitoring
        self.topic_active = False
        self.topic_check_timer = rospy.Timer(rospy.Duration(2.0), self.check_topic_status)
        
    def check_topic_status(self, event):
        """Check if data_topic is being published"""
        try:
            published_topics = [topic for topic, _ in rospy.get_published_topics()]
            is_published = self.resolved_topic in published_topics
            
            # State change detection
            if is_published and not self.topic_active:
                self.topic_active = True
                rospy.loginfo(f"\033[92m[BufferHandler::{self.data_topic}] Input topic is being published. Node ACTIVE\033[0m")
            elif not is_published and self.topic_active:
                self.topic_active = False
                rospy.logwarn(f"\033[91m[BufferHandler::{self.data_topic}] Input topic is NOT being published. Node in STANDBY\033[0m")
            elif not is_published and not self.topic_active:
                # Still not published, log once on first check
                if event.last_expected is None:  # First run
                    rospy.logwarn(f"\033[91m[BufferHandler::{self.data_topic}] No input topic is being published. Node in STANDBY\033[0m")
        except Exception as e:
            rospy.logwarn_throttle(30.0, f"[BufferHandler::{self.data_topic}] Error checking topic status: {e}")
    
    def recording_control_callback(self, msg):
        """Callback for centralized recording enable/disable"""
        if self.store_data:
            if self.recording_enabled != msg.data:
                self.recording_enabled = msg.data
                
                if msg.data:
                    # Recording enabled: create storage folder if not created yet
                    if not self.storage_path_created:
                        try:
                            Path(self.storage_path).mkdir(parents=True, exist_ok=True)
                            self.storage_path_created = True
                            rospy.loginfo(f"[BufferHandler::{self.data_topic}] Created storage folder: {self.storage_path}")
                        except Exception as e:
                            rospy.logerr(f"[BufferHandler::{self.data_topic}] Failed to create storage folder: {e}")
                            self.recording_enabled = False
                            return
                    rospy.loginfo(f"[BufferHandler::{self.data_topic}] Recording ENABLED → storing to: {self.storage_path}")
                else:
                    # Recording disabled
                    rospy.loginfo(f"[BufferHandler::{self.data_topic}] Recording DISABLED")
    
    def _initialize_sync_publisher(self, msg_raw):
        """Initialize the sync publisher with the correct message type on first message"""
        if self.sync_pub is None:
            try:
                msg_type_str = msg_raw._connection_header['type']
                self.sync_msg_class = self._get_message_class(msg_type_str)
                self.sync_pub = rospy.Publisher(self.sync_topic, self.sync_msg_class, queue_size=10, latch=False)
                rospy.loginfo(f"[BufferHandler::{self.data_topic}] Initialized sync publisher with type: {msg_type_str}")
            except Exception as e:
                rospy.logerr(f"[BufferHandler::{self.data_topic}] Failed to initialize sync publisher: {e}")
                # Fallback to AnyMsg if type detection fails
                self.sync_pub = rospy.Publisher(self.sync_topic, rospy.AnyMsg, queue_size=10)
    
    def get_timestamp_from_msg(self, msg):
        """Extract timestamp from message.
        Priority: 1) deserialized header.stamp, 2) raw buffer bytes, 3) rospy.Time.now()
        """
        # 1) Deserialized message with header
        try:
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                return msg.header.stamp.to_nsec()
        except:
            pass
        
        # 2) Raw AnyMsg: extract header.stamp from serialized bytes
        #    ROS messages starting with Header (or containing a msg with Header as first field)
        #    have stamp at bytes 4-11: [seq(4B)][stamp.sec(4B)][stamp.nsec(4B)]
        try:
            if hasattr(msg, '_buff') and len(msg._buff) >= 12:
                sec, nsec = struct.unpack_from('<II', msg._buff, 4)
                # Sanity check: epoch timestamp between 2020 and 2040
                if 1577836800 < sec < 2208988800:
                    return sec * 1000000000 + nsec
        except:
            pass
        
        # 3) Fallback
        return rospy.Time.now().to_nsec()
    
    def get_exposure_from_msg(self, msg_raw):
        """Extract exposure_time from ImageWithMetadata message if available"""
        # Return parameter value as default
        if self.exposure_time_ns > 0:
            return self.exposure_time_ns
        
        # Try to extract from ImageWithMetadata
        if self.handler_type == 'image':
            try:
                from multiespectral_acquire.msg import ImageWithMetadata
                img_msg = ImageWithMetadata()
                img_msg.deserialize(msg_raw._buff)
                
                if hasattr(img_msg.metadata, 'exposure_time') and img_msg.metadata.exposure_time > 0:
                    return img_msg.metadata.exposure_time
            except Exception as e:
                rospy.logwarn_throttle(30, f"[BufferHandler::{self.data_topic}] Could not extract exposure_time from message: {e}")
        
        return 0  # No exposure info available
    
    def data_callback(self, msg_raw):
        """Generic callback for any message type"""
        # Skip if topic not active (in standby)
        if not self.topic_active:
            return
        
        # Initialize sync publisher on first message
        self._initialize_sync_publisher(msg_raw)
        
        # Get timestamp from message
        timestamp = self.get_timestamp_from_msg(msg_raw)
        
        # If store_all mode, handle directly (master needs to be stored but not synchronized)
        if self.store_all:
            # Deserialize
            msg_class = self._get_message_class(msg_raw._connection_header['type'])
            msg = msg_class().deserialize(msg_raw._buff)
            
            # Always republish synchronized message (regardless of recording state)
            if self.sync_pub is not None:
                try:
                    self.sync_pub.publish(msg)
                except Exception as e:
                    rospy.logwarn_throttle(10.0, f"[BufferHandler::{self.data_topic}] Failed to republish: {e}")
            
            # Store to disk only if recording enabled
            if self.recording_enabled and self.store_data:
                if not self.storage_path_created:
                    rospy.logwarn_throttle(5.0, f"[BufferHandler::{self.data_topic}] Recording enabled but storage path not created yet")
                    return
                
                # Determine base_name - priority order:
                # 1. frame_id (embedded by buffer_sync in the pipeline)
                # 2. metadata.img_name (ImageWithMetadata messages)
                # 3. timestamp fallback
                base_name = None
                
                # Try frame_id first (used for sync→crop→store pipeline)
                if hasattr(msg, 'header') and msg.header.frame_id:
                    # frame_id should contain the img_name if set by buffer_sync
                    frame_id = msg.header.frame_id
                    # Validate it looks like a proper name (not a TF frame like "ouster")
                    if frame_id and not frame_id.startswith(('base_', 'ouster', 'map', 'world')):
                        base_name = frame_id
                
                # Try metadata.img_name (ImageWithMetadata)
                if base_name is None and hasattr(msg, 'metadata') and hasattr(msg.metadata, 'img_name'):
                    base_name = msg.metadata.img_name
                
                # Fallback to timestamp
                if base_name is None:
                    if hasattr(msg, 'header') and msg.header.stamp.to_nsec() > 0:
                        ts_nsec = msg.header.stamp.to_nsec()
                    else:
                        ts_nsec = timestamp
                    base_name = f"{ts_nsec}"
                    rospy.logdebug_throttle(10.0, f"[BufferHandler::{self.data_topic}] Using timestamp as filename")
                
                self.store_message(msg, base_name)
        else:
            # Buffer mode: always buffer for potential sync
            self.buffer.add(timestamp, msg_raw)
    
    def master_callback(self, msg):
        """Callback for master trigger"""
        # Skip if topic not active (in standby)
        if not self.topic_active:
            return
        
        # Use camera_timestamp from header.stamp (ROS convention)
        camera_timestamp = msg.metadata.header.stamp.to_nsec()
        
        # Try to get exposure_time from slave messages in buffer using the new helper function
        # This will auto-detect from message or use parameter fallback
        if len(self.buffer.buffer) > 0:
            sample_msg_raw = self.buffer.buffer[0]['data']
            
            # Initialize sync publisher on first master callback with buffered data
            self._initialize_sync_publisher(sample_msg_raw)
            
            exposure_time = self.get_exposure_from_msg(sample_msg_raw)
            
            if exposure_time > 0 and self.exposure_time_ns == 0:
                # Log only once when auto-detecting
                rospy.loginfo_once(f"[BufferHandler::{self.data_topic}] AUTO-DETECTED exposure_time: {exposure_time}ns ({exposure_time/1e6:.2f}ms)")
        else:
            exposure_time = self.exposure_time_ns
        
        # Calculate half_exposure for synchronization
        # This is the optimal sync point: camera_timestamp + exposure_time / 2
        if exposure_time > 0:
            sync_timestamp = camera_timestamp + (exposure_time // 2)
        else:
            # Fallback to camera_timestamp if no exposure configured
            rospy.logwarn_once(f"[BufferHandler::{self.data_topic}] No exposure_time available, using camera_timestamp directly")
            sync_timestamp = camera_timestamp
        
        base_name = msg.metadata.img_name
        
        closest = self.buffer.find_closest(sync_timestamp, self.max_time_diff)
        
        if closest is None:
            # Provide detailed diagnostics
            buffer_size = len(self.buffer.buffer)
            time_since_update = self.buffer.get_time_since_last_update()
            
            if buffer_size == 0:
                if time_since_update is None:
                    rospy.logwarn(f"[BufferHandler::{self.data_topic}] Buffer empty - no messages received yet")
                else:
                    rospy.logwarn(f"[BufferHandler::{self.data_topic}] Buffer empty - not updated for {time_since_update:.1f}s (possible source node crash)")
            else:
                # Find actual closest message and calculate time difference
                actual_closest = min(self.buffer.buffer, key=lambda x: abs(x['timestamp'] - sync_timestamp))
                time_diff_ms = abs(actual_closest['timestamp'] - sync_timestamp) / 1e6
                update_info = f", not updated for {time_since_update:.1f}s" if time_since_update and time_since_update > 2.0 else ""
                rospy.logwarn(f"[BufferHandler::{self.data_topic}] No match found - buffer_size={buffer_size}, "
                              f"closest_diff={time_diff_ms:.1f}ms (max_allowed={self.max_time_diff*1000:.0f}ms){update_info}")
            return
        
        # Always republish synchronized message (regardless of recording state)
        msg_deserialized = None
        if self.sync_pub is not None:
            try:
                # Deserialize the message properly
                msg_class = self._get_message_class(closest['data']._connection_header['type'])
                msg_deserialized = msg_class().deserialize(closest['data']._buff)
                
                # Embed img_name in frame_id so downstream nodes can use it
                # This allows the name to travel through the pipeline (sync→crop→store)
                if hasattr(msg_deserialized, 'header'):
                    msg_deserialized.header.frame_id = base_name
                
                self.sync_pub.publish(msg_deserialized)
            except Exception as e:
                rospy.logdebug(f"[BufferHandler::{self.data_topic}] Could not republish sync message: {e}")
        
        # Store to disk only if recording enabled AND store_data is true
        if self.recording_enabled and self.store_data:
            if not self.storage_path_created:
                rospy.logwarn_throttle(5.0, "[BufferHandler::{self.data_topic}] Recording enabled but storage path not created yet")
                return
            
            # Deserialize message if not already done for republishing
            if msg_deserialized is None:
                try:
                    msg_class = self._get_message_class(closest['data']._connection_header['type'])
                    msg_deserialized = msg_class().deserialize(closest['data']._buff)
                except Exception as e:
                    rospy.logerr(f"[BufferHandler::{self.data_topic}] Failed to deserialize message for storage: {e}")
                    return
            
            self.store_message(msg_deserialized, base_name)
    
    def store_message(self, msg, base_name):
        """Store message based on handler type"""
        if self.handler_type == 'image':
            self._store_image_with_metadata(msg, base_name)
        elif self.handler_type == 'simple_image':
            self._store_simple_image(msg, base_name)
        elif self.handler_type == 'pointcloud':
            self._store_pointcloud(msg, base_name)
        else:
            # Generic: convert to YAML
            self._store_generic_yaml(msg, base_name)
    
    def _store_image_with_metadata(self, msg, base_name):
        """Store ImageWithMetadata as PNG + YAML"""
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
            img_file = os.path.join(self.storage_path, f"{base_name}.png")
            cv2.imwrite(img_file, cv_image)
            
            # Convert metadata to YAML
            metadata = self._msg_to_dict(msg.metadata)
            yaml_file = os.path.join(self.storage_path, f"{base_name}.yaml")
            with open(yaml_file, 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False)
            
            rospy.logdebug(f"[BufferHandler::{self.data_topic}] Stored image: {img_file}")
        except Exception as e:
            rospy.logerr(f"[BufferHandler::{self.data_topic}] Failed to store image: {e}")
    
    def _store_simple_image(self, msg, base_name):
        """Store sensor_msgs/Image as PNG + minimal YAML"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            img_file = os.path.join(self.storage_path, f"{base_name}.png")
            cv2.imwrite(img_file, cv_image)
            
            # Minimal metadata
            metadata = {
                'timestamp': msg.header.stamp.to_nsec(),
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'frame_id': msg.header.frame_id
            }
            yaml_file = os.path.join(self.storage_path, f"{base_name}.yaml")
            with open(yaml_file, 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False)
            
            rospy.logdebug(f"[BufferHandler::{self.data_topic}] Stored simple image: {img_file}")
        except Exception as e:
            rospy.logerr(f"[BufferHandler::{self.data_topic}] Failed to store simple image: {e}")
    
    def _store_pointcloud(self, msg, base_name):
        """Store PointCloud2 as binary + YAML"""
        try:
            # Store as binary (compatible format)
            bin_file = os.path.join(self.storage_path, f"{base_name}.bin")
            
            # Extract points
            points = []
            for point in pc2.read_points(msg, skip_nans=True):
                points.append(point)
            
            # Save as numpy binary (universal format)
            points_array = np.array(points, dtype=np.float32)
            points_array.tofile(bin_file)
            
            # Metadata
            metadata = {
                'timestamp': msg.header.stamp.to_nsec(),
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
                'point_step': msg.point_step,
                'row_step': msg.row_step,
                'is_dense': msg.is_dense,
                'is_bigendian': msg.is_bigendian,
                'fields': [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype, 'count': f.count} 
                          for f in msg.fields]
            }
            yaml_file = os.path.join(self.storage_path, f"{base_name}.yaml")
            with open(yaml_file, 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False)
            
            rospy.logdebug(f"[BufferHandler::{self.data_topic}] Stored pointcloud: {bin_file}")
        except Exception as e:
            rospy.logerr(f"[BufferHandler::{self.data_topic}] Failed to store pointcloud: {e}")
    
    def _store_generic_yaml(self, msg, base_name):
        """Store any message as YAML (universal format)"""
        try:
            data = self._msg_to_dict(msg)
            yaml_file = os.path.join(self.storage_path, f"{base_name}.yaml")
            with open(yaml_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            rospy.logdebug(f"[BufferHandler::{self.data_topic}] Stored generic YAML: {yaml_file}")
        except Exception as e:
            rospy.logerr(f"[BufferHandler::{self.data_topic}] Failed to store generic YAML: {e}")
    
    def _msg_to_dict(self, msg):
        """Convert ROS message to Python dict recursively"""
        if msg is None:
            return None
        
        # Handle basic types
        if isinstance(msg, (int, float, str, bool)):
            return msg
        
        # Handle lists/arrays
        if isinstance(msg, (list, tuple)):
            return [self._msg_to_dict(item) for item in msg]
        
        # Handle ROS messages
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            
            # Handle time/duration specially
            if hasattr(value, 'to_nsec'):
                result[slot] = value.to_nsec()
            else:
                result[slot] = self._msg_to_dict(value)
        
        return result
    
    def _get_message_class(self, msg_type):
        """Dynamically import message class from type string"""
        try:
            parts = msg_type.split('/')
            if len(parts) != 2:
                return None
            
            pkg, msg_name = parts
            module = importlib.import_module(f"{pkg}.msg")
            return getattr(module, msg_name)
        except Exception as e:
            rospy.logwarn(f"[BufferHandler::{self.data_topic}] Failed to import {msg_type}: {e}")
            return None
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        handler = GenericBufferHandler()
        handler.run()
    except rospy.ROSInterruptException:
        pass
