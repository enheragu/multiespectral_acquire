#!/usr/bin/env python3
# encoding: utf-8

import os
import threading
import signal

from flask import Flask, render_template, request, jsonify, make_response, send_from_directory, Response
from flask_socketio import SocketIO, emit

try:
    import rclpy
    from multiespectral_acquire_gui.multiespectral_ros_ac import RosMultiespectralAcquire as MultiespectralAcquire
    using_ros = True
    print("[MultiespectralAcquireGui] Using ROS2 Multiespectral Acquire.")
except ImportError as e:
    print(f"[MultiespectralAcquireGui] ROS2 loading problem: {e}")
    from multiespectral_acquire_gui.multiespectral_dummy_ac import DummyMultiespectralAcquire as MultiespectralAcquire
    using_ros = False
    print("[MultiespectralAcquireGui] No ROS2 detected. Using Dummy Multiespectral Acquire.")


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Resolve package data dir: share/ when installed, __file__ dir when editable.
try:
    from ament_index_python.packages import get_package_share_directory
    _pkg_dir = get_package_share_directory('multiespectral_acquire_gui')
except Exception:
    _pkg_dir = BASE_DIR

app = Flask(__name__,
            template_folder=os.path.join(_pkg_dir, 'templates'),
            static_folder=os.path.join(_pkg_dir, 'static'))
# Cache CSS/JS for an hour. Flask's default (Cache-Control: no-cache) makes the
# browser revalidate every asset on every load; over a relayed Husarnet link that
# round-trip often drops → the page renders unstyled. With a max-age, one good
# load keeps it styled (no re-request). Hard-refresh (Ctrl+Shift+R) bypasses it
# when developing the CSS.
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 3600
# manage_session=False avoids the Flask-SocketIO 5.3.6 + Flask 3.x crash
# (`ctx.session = ...` -> AttributeError: RequestContext.session has no setter),
# which silently kills every client->server event (e.g. change_lidar_topic).
# This GUI does not use Flask sessions, so disabling session management is safe.
socketio = SocketIO(app, async_mode='threading', manage_session=False)

# Shared design system from hitos_setup/web_manager/static.
# Source path takes priority (always up-to-date during development).
_hitos_static = os.path.normpath(os.path.join(BASE_DIR, '..', '..', '..', 'hitos_setup', 'web_manager', 'static'))
if not os.path.isdir(_hitos_static):
    try:
        _hitos_static = os.path.join(get_package_share_directory('hitos_setup'), 'web_manager', 'static')
    except Exception:
        pass
print(f'[MultiespectralAcquireGui] Package dir:   {_pkg_dir}')
print(f'[MultiespectralAcquireGui] Shared static: {_hitos_static}')


@app.route('/hitos_static/<path:filename>')
def hitos_static(filename):
    return send_from_directory(_hitos_static, filename, max_age=3600)

camera_handler = None


@app.route('/')
def home():
    lidar_options = []
    current_lidar_topic = ''
    if camera_handler and using_ros:
        lidar_options = getattr(camera_handler, 'lidar_topic_options', [])
        current_lidar_topic = getattr(camera_handler, 'current_lidar_topic', '')

    return render_template('index.html',
                           frame_rate_lwir=0,
                           frame_rate_rgb=0,
                           total_images_received_lwir=0,
                           total_images_received_rgb=0,
                           lwir_img_path='',
                           rgb_img_path='',
                           recording_enabled=False,
                           lidar_topic_options=lidar_options,
                           current_lidar_topic=current_lidar_topic)


@app.route('/start', methods=['POST'])
def start_recording():
    if camera_handler is None:
        return jsonify({'status': 'not_initialized'})
    camera_handler.enable_recording()
    return jsonify({'status': 'recording_enabled'})


@app.route('/stop', methods=['POST'])
def stop_recording():
    if camera_handler:
        camera_handler.disable_recording()
    return jsonify({'status': 'recording_disabled'})


@app.route('/toggle', methods=['POST'])
def toggle_recording():
    if camera_handler:
        camera_handler.toggle_recording()
    return jsonify({'status': 'toggled'})


@app.route('/manifest')
def manifest():
    res = make_response(render_template('manifest.appcache'), 200)
    res.headers['Content-Type'] = 'text/cache-manifest'
    return res


@app.route('/stream/<name>')
def stream(name):
    # MJPEG (multipart/x-mixed-replace): serves only the LATEST frame and is lossy
    # for slow clients — the server never queues frames, so the old Socket.IO
    # image-emit memory leak (unbounded send queue for a slow Husarnet client)
    # cannot happen. Stats still go over Socket.IO.
    if (camera_handler is None or not hasattr(camera_handler, 'get_frame')
            or name not in ('camera1', 'camera2', 'lidar')):
        return ('stream unavailable', 404)

    def gen():
        last_seq = -1
        while True:
            frame, seq = camera_handler.get_frame(name)
            if frame is not None and seq != last_seq:
                last_seq = seq
                yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                       + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
            socketio.sleep(0.05)

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@socketio.on('image_size')
def handle_image_size(size_data):
    if camera_handler:
        camera_handler.update_image_size(size_data)


@socketio.on('change_lidar_topic')
def handle_change_lidar_topic(data):
    if camera_handler:
        camera_handler.change_lidar_topic(data)


def _sigint_handler(sig, frame):
    global camera_handler
    print("[MultiespectralAcquireGui] SIGINT received.")
    if using_ros and camera_handler:
        camera_handler.stop()
    os._exit(0)


def main():
    signal.signal(signal.SIGINT, _sigint_handler)
    global camera_handler

    if using_ros:
        rclpy.init()
        camera_handler = MultiespectralAcquire(socketio)

        # Flask params come from the ROS2 node parameters
        p = lambda name: camera_handler.get_parameter(name).get_parameter_value()
        flask_host = p('flask_host').string_value
        flask_port = p('flask_port').integer_value

        print(f"[MultiespectralAcquireGui] Flask server on {flask_host}:{flask_port}")

        flask_thread = threading.Thread(
            target=lambda: socketio.run(
                app, host=flask_host, port=flask_port,
                debug=False, use_reloader=False, allow_unsafe_werkzeug=True),
            daemon=True)
        flask_thread.start()

        try:
            rclpy.spin(camera_handler)
        finally:
            camera_handler.stop()
            camera_handler.destroy_node()
            rclpy.shutdown()
    else:
        flask_host = os.environ.get('FLASK_HOST', '0.0.0.0')
        flask_port = int(os.environ.get('FLASK_PORT', 5000))
        camera_handler = MultiespectralAcquire(socketio)
        socketio.run(app, host=flask_host, port=flask_port,
                     debug=False, use_reloader=False, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
