#!/usr/bin/env python3
# encoding: utf-8

from flask_socketio import SocketIO, emit
import cv2
import base64
import time
import os

from FreqCounter import FreqCounter

frame_rate_lwir = FreqCounter()
frame_rate_rgb = FreqCounter()
lwir_img_path = ""
rgb_img_path = ""
lwir_img_storepath = ""
rgb_img_storepath = ""
store_in_drive = False
camera_handler = None

# Camera Actions handling
class DummyMultiespectralAcquire:
    def __init__(self, socketio):
        self.running = False
        self.socketio = socketio

    def sendGoal(self, store):
        print(f"[MultiespectralAcquireGui] Send goal with flag as: {store =}")
        
        frame_rate_lwir.start()
        frame_rate_rgb.start()

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
        global total_images_received_lwir, total_images_received_rgb
        global frame_rate_lwir, frame_rate_rgb, lwir_img_path, rgb_img_path
        global total_images_received_lwir, total_images_received_rgb

        script_path = os.path.dirname(os.path.abspath(__file__))
        lwir_path = os.path.join(script_path, 'images/lwir')
        rgb_path = os.path.join(script_path, 'images/visible')
        
        archivos_dir1 = sorted(os.listdir(lwir_path))
        archivos_dir2 = sorted(os.listdir(rgb_path))
        
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
            
            self.socketio.emit('update_data', {
                'total_images_received_lwir': frame_rate_lwir.cuontItems(),
                'total_images_received_rgb': frame_rate_rgb.cuontItems(),
                'lwir_img_path': lwir_img_path,
                'rgb_img_path': rgb_img_path,
                'lwir_img_storepath': lwir_img_storepath,
                'rgb_img_storepath': rgb_img_storepath,
                'frame_rate_lwir': str(frame_rate_lwir),
                'frame_rate_rgb': str(frame_rate_rgb)
            })

            print(f"Total received lwir: {frame_rate_lwir.cuontItems()}")
            print(f"Total received rgb: {frame_rate_rgb.cuontItems()}")
            print(f"{lwir_img_storepath = }")
            print(f"{rgb_img_storepath = }")

            if not self.running:
                break
            time.sleep(0.15)

    def stop(self):
        self.running = False
        print("[MultiespectralAcquireGui] Finished image acquisition")