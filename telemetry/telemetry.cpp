#include <Preferences.h>
#include "protocol.h"
#include "telemetry.h"
#include "debug.h"

void Telemetry::copyToIncoming(uint8_t* data, size_t len, SerialSource source) {
  incoming.copyFrom(data, len);
  incomingSource = source;
}

Sensor* Telemetry::updateSensor(uint8_t physicalId, uint16_t sensorId, uint8_t subId, uint32_t sensorData, SensorDataType sensorDataType) {
  Sensor* sensor = getSensor(physicalId, sensorId, subId);
  if (sensor == nullptr) {
    if (numSensors < MAX_SENSORS) {
      // add new
      int16_t idx = numSensors++;
      sensor = &(sensors[idx]);
      sensor->_index = idx;
      sensor->physicalId = physicalId;
      sensor->sensorId = sensorId;
      sensor->info = protocolGetSensorInfo(sensorId, subId);
    } else {
      return nullptr;
    }
  }
  sensor->setValue(sensorData, sensorDataType);
  return sensor;
}

Sensor* Telemetry::getSensor(uint8_t physicalId, uint16_t sensorId, uint8_t subId) {
  for (int i=0; i<numSensors; i++) {
    if (sensors[i].sensorId == sensorId && sensors[i].physicalId == physicalId && (!sensors[i].info || sensors[i].info->subId == subId)) {
      return &(sensors[i]);
    }
  }
  return nullptr;
}

static void loadPreferenceString(Preferences& prefs, const char* key, char* value, int maxSize, const char* defaultValue = "") {
  if (!prefs.getString(key, value, maxSize)) {
    strncpy_s(value, defaultValue, maxSize);
  }
}

void Telemetry::load() {
  Preferences preferences;
  if (!preferences.begin("telem")) {
    LOGE("Could not open preferences!");
    return;
  }

  // emergency reset
  //preferences.clear();

  // max key length is 15!!!
  config.input.source = (SerialSource) preferences.getShort("inputSource", SOURCE_UART);
  loadPreferenceString(preferences, "btSource", config.input.btAddress, BD_ADDR_SIZE);
  config.input.protocol = (TelemetryProtocol) preferences.getShort("protocol", PROTOCOL_SMART_PORT);
  config.usb.mode = (SerialMode) preferences.getShort("usbMode", MODE_PASS_THRU);
  loadPreferenceString(preferences, "btName", config.bt.name, NAME_SIZE, "Telemetry BT");
  config.bt.mode = (SerialMode) preferences.getShort("btMode", MODE_DISABLED);
  loadPreferenceString(preferences, "bleName", config.ble.name, NAME_SIZE, "Telemetry BLE");
  config.ble.mode = (SerialMode) preferences.getShort("bleMode", MODE_FILTER);
  loadPreferenceString(preferences, "wifiApHostname", config.wifi.ap.hostname, NAME_SIZE, "telemetry");
  loadPreferenceString(preferences, "wifiApSsid", config.wifi.ap.ssid, SSID_SIZE, "Telemetry WiFi");
  loadPreferenceString(preferences, "wifiApPassword", config.wifi.ap.password, PASSWORD_SIZE);
  loadPreferenceString(preferences, "wifiHostname", config.wifi.client.hostname, NAME_SIZE, "telemetry");
  loadPreferenceString(preferences, "wifiStaSsid", config.wifi.client.remote.ssid, SSID_SIZE);
  loadPreferenceString(preferences, "wifiStaPassword", config.wifi.client.remote.password, PASSWORD_SIZE);
  config.wifi.mode = (WiFiMode_t) preferences.getShort("wifiMode", WIFI_AP);
  loadPreferenceString(preferences, "mapTiles", config.map.tiles, URL_SIZE);
  loadPreferenceString(preferences, "mapApiKey", config.map.apiKey, API_KEY_SIZE);
  loadPreferenceString(preferences, "mqttBroker", config.mqtt.broker, ENDPOINT_SIZE);
  config.mqtt.port = preferences.getShort("mqttPort", 8883);
  loadPreferenceString(preferences, "mqttTopic", config.mqtt.topic, TOPIC_SIZE);
  loadPreferenceString(preferences, "soHostname", config.socket.client.hostname, NAME_SIZE);
  config.socket.client.port = preferences.getShort("soClientPort", 9878);
  config.socket.server.port = preferences.getShort("soServerPort", 9878);
  config.socket.server.mode = (SerialMode) preferences.getShort("soMode", MODE_DISABLED);
  config.internalSensors.enableHallEffect = preferences.getBool("internalSensors", true);
  preferences.end();

  LOGD("BT source address: '%s'", config.input.btAddress);
  LOGD("BT name: '%s'", config.bt.name);
  LOGD("BLE name: '%s'", config.ble.name);
  LOGD("WiFi AP hostname: '%s'", config.wifi.ap.hostname);
  LOGD("WiFi AP SSID: '%s'", config.wifi.ap.ssid);
  LOGD("WiFi AP password: '%s'", config.wifi.ap.password);
  LOGD("WiFi hostname: '%s'", config.wifi.client.hostname);
  LOGD("WiFi station SSID: '%s'", config.wifi.client.remote.ssid);
  LOGD("WiFi station password: '%s'", config.wifi.client.remote.password);
  LOGD("Map tiles: '%s'", config.map.tiles);
  LOGD("Map API key: '%s'", config.map.apiKey);
  LOGD("MQTT broker: '%s'", config.mqtt.broker);
  LOGD("MQTT port: %d", config.mqtt.port);
  LOGD("MQTT topic: '%s'", config.mqtt.topic);
  LOGD("Socket client hostname: '%s'", config.socket.client.hostname);
  LOGD("Socket client port: %d", config.socket.client.port);
  LOGD("Socket server port: %d", config.socket.server.port);
}

void Telemetry::save() {
  Preferences preferences;
  if (!preferences.begin("telem")) {
    LOGE("Could not open preferences!");
    return;
  }

  // max key length is 15!!!
  preferences.putShort("inputSource", config.input.source);
  preferences.putString("btSource", config.input.btAddress);
  preferences.putShort("protocol", config.input.protocol);
  preferences.putShort("usbMode", config.usb.mode);
  preferences.putString("btName", config.bt.name);
  preferences.putShort("btMode", config.bt.mode);
  preferences.putString("bleName", config.ble.name);
  preferences.putShort("bleMode", config.ble.mode);
  preferences.putString("wifiApHostname", config.wifi.ap.hostname);
  preferences.putString("wifiApSsid", config.wifi.ap.ssid);
  preferences.putString("wifiApPassword", config.wifi.ap.password);
  preferences.putString("wifiHostname", config.wifi.client.hostname);
  preferences.putString("wifiStaSsid", config.wifi.client.remote.ssid);
  preferences.putString("wifiStaPassword", config.wifi.client.remote.password);
  preferences.putShort("wifiMode", config.wifi.mode);
  preferences.putString("mapTiles", config.map.tiles);
  preferences.putString("mapApiKey", config.map.apiKey);
  preferences.putString("mqttBroker", config.mqtt.broker);
  preferences.putShort("mqttPort", config.mqtt.port);
  preferences.putString("mqttTopic", config.mqtt.topic);
  preferences.putString("soHostname", config.socket.client.hostname);
  preferences.putShort("soClientPort", config.socket.client.port);
  preferences.putShort("soServerPort", config.socket.server.port);
  preferences.putShort("soMode", config.socket.server.mode);
  preferences.putBool("internalSensors", config.internalSensors.enableHallEffect);
  preferences.end();
}
