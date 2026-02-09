#!/usr/bin/env python3
"""
ROS node for reading temperature and humidity from ESP8266 + DHT22 via serial.
Automatically detects the serial port by looking for common ESP8266 USB chips.
"""

import rospy
import serial
import serial.tools.list_ports
import re
from temperature_driver.msg import TemperatureHumidity


# Common USB VID:PID for ESP8266 boards
ESP8266_USB_IDS = [
    (0x1A86, 0x7523),  # CH340
    (0x10C4, 0xEA60),  # CP2102
    (0x0403, 0x6001),  # FTDI FT232
    (0x1A86, 0x55D4),  # CH9102
]


def find_esp8266_port():
    """
    Auto-detect the ESP8266 serial port by USB VID:PID.
    Returns the device path or None if not found.
    """
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        for vid, pid in ESP8266_USB_IDS:
            if port.vid == vid and port.pid == pid:
                rospy.loginfo(f"Found ESP8266 on {port.device} ({port.description})")
                return port.device
    
    # Fallback: check dmesg for CH340/CP210x
    try:
        import subprocess
        dmesg = subprocess.check_output(['dmesg'], stderr=subprocess.DEVNULL, text=True)
        # Look for CH340 or CP210x attached
        match = re.search(r'(ch34[0-9]|cp210x).*attached to (ttyUSB\d+)', dmesg, re.IGNORECASE)
        if match:
            port = f"/dev/{match.group(2)}"
            rospy.loginfo(f"Found ESP8266 via dmesg on {port}")
            return port
    except Exception as e:
        rospy.logwarn(f"dmesg fallback failed: {e}")
    
    return None


def parse_sensor_data(line):
    """
    Parse the JSON-like format: {temperature:XX.XX, humidity:YY.YY}
    Returns (temperature, humidity) tuple or None if parsing fails.
    """
    # Match the format from the ESP8266
    pattern = r'\{temperature:\s*([-\d.]+),\s*humidity:\s*([-\d.]+)\}'
    match = re.match(pattern, line.strip())
    
    if match:
        try:
            temp = float(match.group(1))
            hum = float(match.group(2))
            return (temp, hum)
        except ValueError:
            return None
    return None


class DHT22Node:
    def __init__(self):
        rospy.init_node('dht22', anonymous=False)
        
        # Parameters
        self.port = rospy.get_param('~port', '')  # Empty = auto-detect
        self.baudrate = rospy.get_param('~baudrate', 74880)
        self.frame_id = rospy.get_param('~frame_id', 'dht22_link')
        self.retry_interval = rospy.get_param('~retry_interval', 5.0)
        
        # Publisher
        self.pub = rospy.Publisher('~data', TemperatureHumidity, queue_size=10)
        
        self.serial = None
        
    def connect(self):
        """Establish serial connection, with auto-detection if needed."""
        port = self.port
        
        if not port:
            port = find_esp8266_port()
            if not port:
                rospy.logwarn("ESP8266 not found, will retry...")
                return False
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=2.0
            )
            rospy.loginfo(f"Connected to {port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open {port}: {e}")
            return False
    
    def run(self):
        """Main loop: read serial data and publish."""
        rate = rospy.Rate(10)  # Check at 10Hz, sensor sends at 1Hz
        
        while not rospy.is_shutdown():
            # Try to connect if not connected
            if self.serial is None or not self.serial.is_open:
                if not self.connect():
                    rospy.sleep(self.retry_interval)
                    continue
            
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore')
                    
                    # Skip debug/status messages
                    if 'ERROR' in line or '===' in line or 'Ready' in line:
                        rospy.logdebug(f"ESP8266: {line.strip()}")
                        continue
                    
                    data = parse_sensor_data(line)
                    if data:
                        msg = TemperatureHumidity()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = self.frame_id
                        msg.temperature = data[0]
                        msg.humidity = data[1]
                        self.pub.publish(msg)
                        rospy.logdebug(f"T={data[0]:.1f}°C, H={data[1]:.1f}%")
                        
            except serial.SerialException as e:
                rospy.logerr(f"Serial error: {e}")
                self.serial = None
                rospy.sleep(self.retry_interval)
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
            
            rate.sleep()
        
        if self.serial and self.serial.is_open:
            self.serial.close()


if __name__ == '__main__':
    try:
        node = DHT22Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
