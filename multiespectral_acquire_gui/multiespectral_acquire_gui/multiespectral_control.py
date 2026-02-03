#!/usr/bin/env python3
# encoding: utf-8

import os
import threading
import signal

from flask import Flask, render_template, request, jsonify, make_response
from flask_socketio import SocketIO, emit

try:
    import rospy
    from multiespectral_acquire_gui.multiespectral_ros_ac import RosMultiespectralAcquire as MultiespectralAcquire
    using_ros = True
    print(f"[MultiespectralAcquireGui] Using ROS Multiespectral Acquire.")
except ImportError as e:
    print(f"[MultiespectralAcquireGui] ROS loading problem: {e}")
    from multiespectral_acquire_gui.multiespectral_dummy_ac import DummyMultiespectralAcquire as MultiespectralAcquire
    using_ros = False
    print(f"[MultiespectralAcquireGui] No ROS detected. Using Dummy Multiespectral Acquire.")


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, 
           template_folder=os.path.join(BASE_DIR, 'templates'),
           static_folder=os.path.join(BASE_DIR, 'static'))
socketio = SocketIO(app, async_mode='threading')

camera_handler = None

@app.route('/')
def home():
    lidar_options = []
    current_lidar_topic = ''
    if camera_handler and using_ros:
        try:
            from multiespectral_acquire_gui.multiespectral_ros_ac import lidar_topic_options
            lidar_options = lidar_topic_options
            current_lidar_topic = camera_handler.current_lidar_topic if hasattr(camera_handler, 'current_lidar_topic') else ''
        except:
            pass
    
    return render_template('index.html', frame_rate_lwir=0, 
                           frame_rate_rgb=0,
                           total_images_received_lwir=0,
                           total_images_received_rgb=0,
                           lwir_img_path="", 
                           rgb_img_path="",
                           recording_enabled=False,
                           lidar_topic_options=lidar_options,
                           current_lidar_topic=current_lidar_topic)

@app.route('/start', methods=['POST'])
def start_recording():
    global camera_handler
    if camera_handler is None:
        return jsonify({"status": "not_initialized"})
    
    camera_handler.enable_recording()
    return jsonify({"status": "recording_enabled"})


@app.route('/stop', methods=['POST'])
def stop_recording():
    global camera_handler
    if camera_handler:
        camera_handler.disable_recording()
    return jsonify({"status": "recording_disabled"})

@app.route('/toggle', methods=['POST'])
def toggle_recording():
    global camera_handler
    if camera_handler:
        camera_handler.toggle_recording()
    return jsonify({"status": "toggled"})

@app.route('/manifest')
def manifest():
    res = make_response(render_template('manifest.appcache'), 200)
    res.headers["Content-Type"] = "text/cache-manifest"
    return res


# SocketIO event handlers
@socketio.on('image_size')
def handle_image_size(size_data):
    global camera_handler
    if camera_handler:
        camera_handler.update_image_size(size_data)

@socketio.on('change_lidar_topic')
def handle_change_lidar_topic(data):
    global camera_handler
    print("\n" + "="*80)
    print("[SOCKETIO] change_lidar_topic event received!")
    print(f"[SOCKETIO] Data: {data}")
    print("="*80 + "\n")
    if camera_handler:
        camera_handler.change_lidar_topic(data)
    else:
        print("[SOCKETIO] ERROR: camera_handler is None!")


def sigint_handler(sig, frame):
    global camera_handler
    print("[MultiespectralAcquireGui] SIGINT received.")
    if using_ros and camera_handler:
        camera_handler.stop()
    exit(0)

def main():
    signal.signal(signal.SIGINT, sigint_handler)
    
    global camera_handler
    
    if using_ros:
        rospy.init_node('multiespectral_flask_gui', anonymous=True)
        
        # Get host and port from ROS parameters (allows configuration via launch file)
        flask_host = rospy.get_param('~flask_host', '0.0.0.0')
        flask_port = rospy.get_param('~flask_port', 5000)
        
        rospy.loginfo(f"[MultiespectralAcquireGui] Starting Flask server on {flask_host}:{flask_port}")
        
        camera_handler = MultiespectralAcquire(socketio)
        flask_thread = threading.Thread(target=lambda: socketio.run(app, host=flask_host, port=flask_port, debug=True, use_reloader=False, allow_unsafe_werkzeug=True), daemon=True)
        flask_thread.start()
        rospy.spin()
    else:
        # Standalone mode: use defaults or environment variables
        flask_host = os.environ.get('FLASK_HOST', '0.0.0.0')
        flask_port = int(os.environ.get('FLASK_PORT', 5000))
        camera_handler = MultiespectralAcquire(socketio)
        socketio.run(app, host=flask_host, port=flask_port, debug=True, use_reloader=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()