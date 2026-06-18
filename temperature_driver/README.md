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
ros2 launch temperature_driver dht22.launch.py
```

**Parameters** (in `launch/dht22.launch.py`):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `$DHT22_PORT` (empty → auto-detect) | USB serial port |
| `baudrate` | `74880` | Serial baud rate |
| `frame_id` | `dht22_link` | TF frame ID |
| `retry_interval` | `5.0` | Seconds between reconnect attempts |

### Message

`msg/TemperatureHumidity.msg` — Custom message with temperature (°C) and relative humidity (%).