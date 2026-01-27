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
# socketio = SocketIO(app, cors_allowed_origins="*") # Allow multiple connections from different origins
socketio = SocketIO(app, async_mode='threading')

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
    if camera_handler is None:
        return jsonify({"status": "not_initialized"})
    
    store_in_drive = 'store_in_drive' in request.form
    camera_handler.sendGoal(store_in_drive)
    return jsonify({"status": "started"})


@app.route('/stop', methods=['POST'])
def stop_camera():
    global camera_handler
    if camera_handler:
        camera_handler.cancelGoal()
    return jsonify({"status": "stopped"})

@app.route('/manifest')
def manifest():
    res = make_response(render_template('manifest.appcache'), 200)
    res.headers["Content-Type"] = "text/cache-manifest"
    return res


def sigint_handler(sig, frame):
    global camera_handler
    print("[MultiespectralAcquireGui] SIGINT received.")
    if using_ros and camera_handler:
        camera_handler.stop()
    socketio.stop()
    exit(0)

def main():
    signal.signal(signal.SIGINT, sigint_handler)
    
    global camera_handler
    
    if using_ros:
        rospy.init_node('multiespectral_flask_gui', anonymous=True)
        camera_handler = MultiespectralAcquire(socketio)
        flask_thread = threading.Thread(target=lambda: socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False, allow_unsafe_werkzeug=True), daemon=True)
        flask_thread.start()
        rospy.spin()
    else:
        camera_handler = MultiespectralAcquire(socketio)
        socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()