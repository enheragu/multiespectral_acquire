#!/usr/bin/env python3
# encoding: utf-8

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import cv2
import base64
import time
import threading
import signal
import os

try: 
    import rospy 
    from multiespectral_ros_ac import RosMultiespectralAcquire as MultiespectralAcquire
    print(f"[MultiespectralAcquireGui] Using ROS Multiespectral Acquire.")
except ImportError: 
    from multiespectral_dummy_ac import DummyMultiespectralAcquire as MultiespectralAcquire
    print(f"[MultiespectralAcquireGui] No ROS detected. Using Dummy Multiespectral Acquire.")

app = Flask(__name__)
# socketio = SocketIO(app, cors_allowed_origins="*") # Allow multiple connections from different origins
socketio = SocketIO(app)

store_in_drive = False
camera_handler = None

@app.route('/')
def home():
    return render_template('index.html', frame_rate_lwir=0, 
                           frame_rate_rgb=0,
                           total_images_received_lwir=0,
                           total_images_received_rgb=0,
                           lwir_img_path="", 
                           rgb_img_path="",
                           lwir_img_storepath="", 
                           rgb_img_storepath="",
                           store_in_drive=False)

@app.route('/start', methods=['POST'])
def start_camera():
    global camera_handler, store_in_drive
    store_in_drive = 'store_in_drive' in request.form
    if not camera_handler:
        camera_handler = MultiespectralAcquire(socketio)
    init_success = camera_handler.sendGoal(store_in_drive)
    if init_success:
        print(f"[MultiespectralAcquireGui] Requested goal with {store_in_drive = }.")
        threading.Thread(target=camera_handler.execute).start()
        return jsonify({"status": "started"})
    else:
        return jsonify({"status": "failed to start"}), 500

@app.route('/stop', methods=['POST'])
def stop_camera():
    global camera_handler
    if camera_handler:
        camera_handler.stop()
        camera_handler = None
    return jsonify({"status": "stopped"})

def sigint_handler(sig, frame):
    print("[MultiespectralAcquireGui] SIGINT received, closing application.")
    global camera_handler
    if camera_handler:
        camera_handler.stop()
    os._exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    print("[MultiespectralAcquireGui] Start camera_handler.")
    camera_handler = MultiespectralAcquire(socketio)
    print("[MultiespectralAcquireGui] Start Flask app.")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
