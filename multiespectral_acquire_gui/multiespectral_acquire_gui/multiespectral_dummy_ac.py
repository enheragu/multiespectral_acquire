#!/usr/bin/env python3
# encoding: utf-8

import cv2
import base64
import time
import os

from multiespectral_acquire_gui.FreqCounter import FreqCounter

frame_rate_lwir = FreqCounter()
frame_rate_rgb = FreqCounter()
frame_rate_lidar = FreqCounter()
lwir_img_path = ""
rgb_img_path = ""
lidar_img_path = ""
lwir_img_storepath = ""
rgb_img_storepath = ""
lidar_img_storepath = ""
store_in_drive = False
camera_handler = None
lidar_available = False  # Can be set based on presence of dummy lidar images


# Camera Actions handling
class DummyMultiespectralAcquire:
    def __init__(self, socketio):
        global lidar_available
        self.running = False
        self.socketio = socketio
        
        # Check if lidar dummy images exist
        script_path = os.path.dirname(os.path.abspath(__file__))
        lidar_path = os.path.join(script_path, 'images/swir')
        lidar_available = os.path.isdir(lidar_path) and len(os.listdir(lidar_path)) > 0
        if lidar_available:
            print(f"[MultiespectralAcquireGui] LiDAR dummy images found at {lidar_path}")
        else:
            print(f"[MultiespectralAcquireGui] No LiDAR dummy images found. LiDAR display disabled.")

    def sendGoal(self, store):
        print(f"[MultiespectralAcquireGui] Send goal with flag as: {store =}")
        
        frame_rate_lwir.start()
        frame_rate_rgb.start()
        if lidar_available:
            frame_rate_lidar.start()

        self.execute()

    def cancelGoal(self):
        pass

    def imgCB(self):
        pass

    def doneCb(self):
        pass

    def activeCb(self):
        pass

    def feedbackCB(self):
        pass

    def execute(self):
        self.running = True
        global frame_rate_lwir, frame_rate_rgb, frame_rate_lidar
        global lwir_img_path, rgb_img_path, lidar_img_path
        global lidar_available

        script_path = os.path.dirname(os.path.abspath(__file__))
        lwir_path = os.path.join(script_path, 'images/lwir')
        rgb_path = os.path.join(script_path, 'images/visible')
        lidar_path = os.path.join(script_path, 'images/swir')
        
        archivos_dir1 = sorted(os.listdir(lwir_path))
        archivos_dir2 = sorted(os.listdir(rgb_path))
        archivos_lidar = sorted(os.listdir(lidar_path)) if lidar_available else []
        
        # Use itertools.cycle for lidar if fewer images, or zip_longest
        from itertools import cycle
        lidar_cycle = cycle(archivos_lidar) if archivos_lidar else None
        
        print(f"[MultiespectralAcquireGui] Start image posting iteration")
        for archivo1, archivo2 in zip(archivos_dir1, archivos_dir2):
            path1 = os.path.join(lwir_path, archivo1)
            path2 = os.path.join(rgb_path, archivo2)

            lwir_image = cv2.imread(path1)
            rgb_image = cv2.imread(path2)

            _, lwir_buffer = cv2.imencode('.jpg', lwir_image)
            lwir_img_path = base64.b64encode(lwir_buffer).decode('utf-8')
            lwir_img_storepath = str(lwir_path)

            _, rgb_buffer = cv2.imencode('.jpg', rgb_image)
            rgb_img_path = base64.b64encode(rgb_buffer).decode('utf-8')
            rgb_img_storepath = str(rgb_path)
            
            frame_rate_lwir.tick()
            frame_rate_rgb.tick()
            
            # Build data dict
            data = {
                'total_images_received_lwir': frame_rate_lwir.countItems(),
                'total_images_received_rgb': frame_rate_rgb.countItems(),
                'lwir_img_path': lwir_img_path,
                'rgb_img_path': rgb_img_path,
                'lwir_img_storepath': lwir_img_storepath,
                'rgb_img_storepath': rgb_img_storepath,
                'frame_rate_lwir': str(frame_rate_lwir),
                'frame_rate_rgb': str(frame_rate_rgb),
                'lidar_available': lidar_available,
            }
            
            # Add lidar data if available
            if lidar_available and lidar_cycle:
                lidar_file = next(lidar_cycle)
                lidar_file_path = os.path.join(lidar_path, lidar_file)
                lidar_image = cv2.imread(lidar_file_path)
                if lidar_image is not None:
                    _, lidar_buffer = cv2.imencode('.jpg', lidar_image)
                    lidar_img_path = base64.b64encode(lidar_buffer).decode('utf-8')
                    lidar_img_storepath = str(lidar_path)
                    frame_rate_lidar.tick()
                    
                    data['total_images_received_lidar'] = frame_rate_lidar.countItems()
                    data['lidar_img_path'] = lidar_img_path
                    data['lidar_img_storepath'] = lidar_img_storepath
                    data['frame_rate_lidar'] = str(frame_rate_lidar)
            
            self.socketio.emit('update_data', data)

            print(f"Total received lwir: {frame_rate_lwir.countItems()}")
            print(f"Total received rgb: {frame_rate_rgb.countItems()}")
            if lidar_available:
                print(f"Total received lidar: {frame_rate_lidar.countItems()}")
            print(f"{lwir_img_storepath = }")
            print(f"{rgb_img_storepath = }")

            if not self.running:
                break
            time.sleep(0.15)

    def stop(self):
        self.running = False
        frame_rate_lwir.stop()
        frame_rate_rgb.stop()
        frame_rate_lidar.stop()
        print("[MultiespectralAcquireGui] Finished image acquisition")