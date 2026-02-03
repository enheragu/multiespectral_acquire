## Temperature Driver

Its a small DHT22 driver that sends temperature/humidity data through serial port from a NodeMCU board. For easyness it is handled thorugh PlatformIO toolchain, with "DHT Sensor Library" from Adafruit as it main requirement and ESP8266 as board (needs extra config from http://arduino.esp8266.com/stable/package_esp8266com_index.json in File>Preferences>Extra URLs, noce added install ESP8266 board).

Includes a ROS publisher that detects USB Port and publish in TemperatureHumidity message.