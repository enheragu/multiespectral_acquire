#!/usr/bin/env python3
"""
ROS2 node for reading temperature and humidity from ESP8266 + DHT22 via serial.
Automatically detects the serial port by looking for common ESP8266 USB chips.
"""

import time
import re
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node

from temperature_driver.msg import TemperatureHumidity


# Common USB VID:PID for ESP8266 boards
ESP8266_USB_IDS = [
    (0x1A86, 0x7523),  # CH340
    (0x10C4, 0xEA60),  # CP2102
    (0x0403, 0x6001),  # FTDI FT232
    (0x1A86, 0x55D4),  # CH9102
]


def find_esp8266_port(logger):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        for vid, pid in ESP8266_USB_IDS:
            if port.vid == vid and port.pid == pid:
                logger.info(f"Found ESP8266 on {port.device} ({port.description})")
                return port.device

    # Fallback: check dmesg for CH340/CP210x
    try:
        import subprocess
        dmesg = subprocess.check_output(['dmesg'], stderr=subprocess.DEVNULL, text=True)
        match = re.search(r'(ch34[0-9]|cp210x).*attached to (ttyUSB\d+)', dmesg, re.IGNORECASE)
        if match:
            port = f"/dev/{match.group(2)}"
            logger.info(f"Found ESP8266 via dmesg on {port}")
            return port
    except Exception as e:
        logger.warn(f"dmesg fallback failed: {e}")

    return None


def parse_sensor_data(line):
    pattern = r'\{temperature:\s*([-\d.]+),\s*humidity:\s*([-\d.]+)\}'
    match = re.match(pattern, line.strip())
    if match:
        try:
            return (float(match.group(1)), float(match.group(2)))
        except ValueError:
            return None
    return None


class DHT22Node(Node):
    def __init__(self):
        super().__init__('dht22')

        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 74880)
        self.declare_parameter('frame_id', 'dht22_link')
        self.declare_parameter('retry_interval', 5.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.retry_interval = self.get_parameter('retry_interval').get_parameter_value().double_value

        self.pub = self.create_publisher(TemperatureHumidity, '~/data', 10)
        self.serial = None
        self._next_retry = 0.0

        # Poll at 10 Hz; the sensor sends at ~1 Hz but we want to drain the buffer promptly
        self.timer = self.create_timer(0.1, self._timer_callback)

    def _connect(self):
        port = self.port or find_esp8266_port(self.get_logger())
        if not port:
            self.get_logger().warn("ESP8266 not found, will retry...")
            return False
        try:
            self.serial = serial.Serial(port=port, baudrate=self.baudrate, timeout=2.0)
            self.get_logger().info(f"Connected to {port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return False

    def _timer_callback(self):
        if self.serial is None or not self.serial.is_open:
            now = time.monotonic()
            if now < self._next_retry:
                return
            if not self._connect():
                self._next_retry = now + self.retry_interval
                return

        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8', errors='ignore')

                if 'ERROR' in line or '===' in line or 'Ready' in line:
                    log = self.get_logger().warn if 'ERROR' in line else self.get_logger().debug
                    log(f"ESP8266: {line.strip()}")
                    return

                data = parse_sensor_data(line)
                if data:
                    msg = TemperatureHumidity()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_id
                    msg.temperature = data[0]
                    msg.humidity = data[1]
                    self.pub.publish(msg)
                    self.get_logger().debug(f"T={data[0]:.1f}°C, H={data[1]:.1f}%")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            if self.serial:
                self.serial.close()
            self.serial = None
            self._next_retry = time.monotonic() + self.retry_interval
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DHT22Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
