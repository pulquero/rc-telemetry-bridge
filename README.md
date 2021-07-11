# RC Telemetry Bridge

Use an ESP32 to consume telemetry from your radio and send it over various transports.

Currently supports FrSky Smart port.

Input:
 - Smart port UART from FrSky compatible radio to ESP32 UART2.
 - Telemetry over Bluetooth (BLE), e.g. X9D+ SE.

Output:
 - USB (compatible with Android telemetry apps).
 - Bluetooth Serial (compatible with Android telemetry apps).
 - BLE (compatible with Android telemetry apps).
 - WiFi (AP and/or station)
   - telemetry dashboard webapp with map,
   - JSON REST API,
   - WebSockets,
   - MQTT (e.g. IoT).
