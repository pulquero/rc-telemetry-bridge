![CI](https://github.com/pulquero/rc-telemetry-bridge/actions/workflows/ci.yml/badge.svg)

# RC Telemetry Bridge

Use an ESP32 to consume telemetry from your radio and send it over various transports.

Supports the following telemetry protocols:
 - FrSky Smart port,
 - TBS Crossfire CRSF,
 - ImmersionRC Ghost,
 - or pass-thru any byte stream!

Input:
 - Smart port UART from FrSky compatible radio to ESP32 UART2.
 - Smart port UART from FrSky R9M to ESP32 UART2.
 - Telemetry over Bluetooth (BLE), e.g. X9D+ SE.
 - Byte stream from a server socket.

Output:
 - USB (compatible with Android telemetry apps).
 - Bluetooth Serial (compatible with Android telemetry apps).
 - BLE (compatible with Android telemetry apps).
 - WiFi (AP and/or station)
   - built-in telemetry dashboard webapp with map,
   - JSON REST API,
   - WebSockets,
   - MQTT (e.g. IoT).
 - Byte stream from a server socket.

## Hardware

Minimum:
 - ESP32 development board.

Optional:
 - 5 LEDs.

## Software dependencies

 - [MQTT](https://github.com/256dpi/arduino-mqtt)
 - [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
 - [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)
 - [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)

## Install

1. Install [Arduino IDE](https://www.arduino.cc/en/software) (version 1.8 or higher required).
2. Run `install_libs.cmd`/`install_libs.sh` to install ESP32 tools and library dependencies.
3. Open `telemetry/telemetry.ino` in Arduino IDE.
4. Select your hardware (e.g. ESP32 Dev Module) under 'Tools' -> 'Board: ...' -> 'ESP32 Arduino'.
5. Change 'Tools' -> 'Partition Scheme' to 'No OTA (2MB APP/2MB SPIFFS)'.
6. Connect your ESP32 board.
7. Change 'Tools' -> 'Port' to the port the ESP32 board is connected to.
8. Run 'Tools' -> 'ESP32 Sketch Data Upload'.
9. Hit the upload button.

## Instructions

The ESP32 should start in WiFi access point mode.
Point a browser at it, click on the 'Settings' link at the bottom of the page, and adjust the configuration appropriately.
Then, reboot.

### Touch pin control

Pin 13 - Toggle WiFi station mode.

Pin 14 - Toggle WiFi access point mode.

Pin 15 - Toggle BLE advertisement.


