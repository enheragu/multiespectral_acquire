# temperature_driver

ROS driver for DHT22 temperature/humidity sensor connected via a NodeMCU (ESP8266) board over USB serial.

## Hardware Setup

- **Sensor**: DHT22 (AM2302) — temperature + humidity
- **Board**: NodeMCU / ESP8266
- **Connection**: USB serial (default baud rate: 74880)

### Firmware (PlatformIO)

The `dht22_PIO/` folder contains the PlatformIO project for the NodeMCU firmware.

**Requirements**:
- PlatformIO toolchain
- Adafruit "DHT Sensor Library"
- ESP8266 board support (add `http://arduino.esp8266.com/stable/package_esp8266com_index.json` in Arduino IDE → File → Preferences → Extra Board Manager URLs, then install ESP8266)

## ROS Node

`scripts/dht22_node.py` — Detects USB port automatically and publishes sensor readings.

### Launch

```bash
roslaunch temperature_driver dht22.launch
```

**Parameters** (in `launch/dht22.launch`):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | auto-detected | USB serial port |
| `baudrate` | `74880` | Serial baud rate |
| `frame_id` | `dht22` | TF frame ID |

### Message

`msg/TemperatureHumidity.msg` — Custom message with temperature (°C) and relative humidity (%).