# RC Telemetry Bridge

Use an ESP32 to consume telemetry from your radio and send it over various transports.

Supports the following telemetry protocols:
 -  FrSky Smart port,
 - TBS Crossfire CRSF.

Input:
 - Smart port UART from FrSky compatible radio to ESP32 UART2.
 - Smart port UART from FrSky R9M to ESP32 UART2.
 - Telemetry over Bluetooth (BLE), e.g. X9D+ SE.

Output:
 - USB (compatible with Android telemetry apps).
 - Bluetooth Serial (compatible with Android telemetry apps).
 - BLE (compatible with Android telemetry apps).
 - WiFi (AP and/or station)
   - built-in telemetry dashboard webapp with map,
   - JSON REST API,
   - WebSockets,
   - MQTT (e.g. IoT).

## Dependencies

 - MQTT
 - NimBLE-Arduino
 - AsyncTCP
 - ESPAsyncWebServer

 
