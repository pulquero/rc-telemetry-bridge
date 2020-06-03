#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "json.h"
#include "web.h"
#include "debug.h"

#define JSON_BUFFER_SIZE 256
#define WS_EMIT_RATE 300

static void wsEventHandler(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
static void sendJson(AsyncWebServerRequest* request);
static void sendSettings(AsyncWebServerRequest* request);
static int writeSensorJson(char* out, const Sensor& sensor, bool all);
static const String indexTemplateProcessor(const String& var);
static const String settingsTemplateProcessor(const String& var);

static Telemetry* _telemetry;

static const String empty("");
static AsyncWebServer* webServer = nullptr;
static AsyncWebSocket* webSocket = nullptr;
static bool isWebRunning = false;

void webBegin(Telemetry* telemetry) {
  if (!webServer) {
    webServer = new AsyncWebServer(80);
    webSocket = new AsyncWebSocket("/ws");
    webSocket->onEvent(wsEventHandler);
    // first match wins
    webServer->addHandler(webSocket);
    webServer->on("/sensors", sendJson);
    webServer->on("/settings.html", sendSettings);
    webServer->serveStatic("/", SPIFFS, "/").setDefaultFile("index.html").setTemplateProcessor(indexTemplateProcessor);
  }
  if (!isWebRunning) {
    _telemetry = telemetry;
    SPIFFS.begin();
    webServer->begin();
    isWebRunning = true;
  }
  LOGMEM();
}

void webStop() {
  if (isWebRunning) {
    webServer->end();
    SPIFFS.end();
    _telemetry = nullptr;
    isWebRunning = false;
  }
  /*
   * currently causes CORRUPT HEAP: Bad head
  if (webServer) {
    delete webServer;
    delete webSocket;
    webSocket = nullptr;
    webServer = nullptr;
  }
  */
  LOGMEM();
}

void wsEventHandler(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      LOGD("WS connected");
      client->_tempObject = new bool[MAX_SENSORS]();
      break;
    case WS_EVT_DISCONNECT:
      LOGD("WS disconnected");
      delete client->_tempObject;
      break;
  }
}

const String indexTemplateProcessor(const String& var) {
  if (var == "MAP_TILES") {
    return _telemetry->config.map.tiles;
  } else if (var == "MAP_API_KEY") {
    return _telemetry->config.map.apiKey;
  } else {
    return empty;
  }
}

void webLoop() {
  static uint32_t lastCleanup = 0;
  if (isWebRunning && (millis() - lastCleanup) > 500) {
    webSocket->cleanupClients();
    lastCleanup = millis();
  }
}

void sendJson(AsyncWebServerRequest* request) {
  AsyncResponseStream* response = request->beginResponseStream("application/json");
  response->print("{\"sensors\": [\n");
  bool isFirst = true;
  for (int i=0; i<_telemetry->numSensors; i++) {
    char json[JSON_BUFFER_SIZE];
    int len = writeSensorJson(json, _telemetry->sensors[i], true);
    if (len > JSON_BUFFER_SIZE) {
      LOGE("JSON buffer size exceeded! (%d)", len);
    }
    if (isFirst) {
      isFirst = false;
    } else {
      response->print(",\n");
    }
    response->print(json);
  }
  response->print("\n]}");
  request->send(response);
}

static String _status = empty;

const String settingsTemplateProcessor(const String& var) {
  static const String checked("checked");
  if (var == "USB_DISABLED") {
    return _telemetry->config.usb.mode == MODE_DISABLED ? checked : empty;
  } else if (var == "USB_PASSTHRU") {
    return _telemetry->config.usb.mode == MODE_PASS_THRU ? checked : empty;
  } else if (var == "USB_FILTER") {
    return _telemetry->config.usb.mode == MODE_FILTER ? checked : empty;
  } else if (var == "BT_NAME") {
    return _telemetry->config.bt.name;
  } else if (var == "BT_DISABLED") {
    return _telemetry->config.bt.mode == MODE_DISABLED ? checked : empty;
  } else if (var == "BT_PASSTHRU") {
    return _telemetry->config.bt.mode == MODE_PASS_THRU ? checked : empty;
  } else if (var == "BT_FILTER") {
    return _telemetry->config.bt.mode == MODE_FILTER ? checked : empty;
  } else if (var == "BLE_NAME") {
    return _telemetry->config.ble.name;
  } else if (var == "BLE_DISABLED") {
    return _telemetry->config.ble.mode == MODE_DISABLED ? checked : empty;
  } else if (var == "BLE_PASSTHRU") {
    return _telemetry->config.ble.mode == MODE_PASS_THRU ? checked : empty;
  } else if (var == "BLE_FILTER") {
    return _telemetry->config.ble.mode == MODE_FILTER ? checked : empty;
  } else if (var == "AP_HOSTNAME") {
    return _telemetry->config.wifi.ap.hostname;
  } else if (var == "AP_SSID") {
    return _telemetry->config.wifi.ap.ssid;
  } else if (var == "AP_PASSWORD") {
    return _telemetry->config.wifi.ap.password;
  } else if (var == "HOSTNAME") {
    return _telemetry->config.wifi.client.hostname;
  } else if (var == "REMOTE_SSID") {
    return _telemetry->config.wifi.client.remote.ssid;
  } else if (var == "REMOTE_PASSWORD") {
    return _telemetry->config.wifi.client.remote.password;
  } else if (var == "MAP_TILES") {
    return _telemetry->config.map.tiles;
  } else if (var == "MAP_API_KEY") {
    return _telemetry->config.map.apiKey;
  } else if (var == "MQTT_BROKER") {
    return _telemetry->config.mqtt.broker;
  } else if (var == "MQTT_PORT") {
    static char szPort[8];
    itoa(_telemetry->config.mqtt.port, szPort, 10);
    return szPort;
  } else if (var == "MQTT_TOPIC") {
    return _telemetry->config.mqtt.topic;
  } else if (var == "INTERNAL_SENSORS") {
    return _telemetry->config.internalSensors.enableHallEffect ? checked : empty;
  } else if (var == "STATUS") {
    return _status;
  } else {
    return empty;
  }
}

void sendSettings(AsyncWebServerRequest* request) {
  static const String savedStatus("Saved");
  _status = empty;
  int numParams = request->params();
  if (numParams > 0) {
    for (int i=0; i<numParams; i++) {
      AsyncWebParameter* param = request->getParam(i);
      String name = param->name();
      String value = param->value();
      if (name == "usb_mode") {
        if (value == "filter") {
          _telemetry->config.usb.mode = MODE_FILTER;
        } else if (value == "passthru") {
          _telemetry->config.usb.mode = MODE_PASS_THRU;
        } else {
          _telemetry->config.usb.mode = MODE_DISABLED;
        }
      } else if (name == "bt_name") {
        strncpy_s(_telemetry->config.bt.name, value.c_str(), NAME_SIZE);
      } else if (name == "bt_mode") {
        if (value == "filter") {
          _telemetry->config.bt.mode = MODE_FILTER;
        } else if (value == "passthru") {
          _telemetry->config.bt.mode = MODE_PASS_THRU;
        } else {
          _telemetry->config.bt.mode = MODE_DISABLED;
        }
      } else if (name == "ble_name") {
        strncpy_s(_telemetry->config.ble.name, value.c_str(), NAME_SIZE);
      } else if (name == "ble_mode") {
        if (value == "filter") {
          _telemetry->config.ble.mode = MODE_FILTER;
        } else if (value == "passthru") {
          _telemetry->config.ble.mode = MODE_PASS_THRU;
        } else {
          _telemetry->config.ble.mode = MODE_DISABLED;
        }
      } else if (name == "ap_hostname") {
        strncpy_s(_telemetry->config.wifi.ap.hostname, value.c_str(), NAME_SIZE);
      } else if (name == "ap_ssid") {
        strncpy_s(_telemetry->config.wifi.ap.ssid, value.c_str(), SSID_SIZE);
      } else if (name == "ap_password") {
        strncpy_s(_telemetry->config.wifi.ap.password, value.c_str(), PASSWORD_SIZE);
      } else if (name == "hostname") {
        strncpy_s(_telemetry->config.wifi.client.hostname, value.c_str(), NAME_SIZE);
      } else if (name == "remote_ssid") {
        strncpy_s(_telemetry->config.wifi.client.remote.ssid, value.c_str(), SSID_SIZE);
      } else if (name == "remote_password") {
        strncpy_s(_telemetry->config.wifi.client.remote.password, value.c_str(), PASSWORD_SIZE);
      } else if (name == "map_tiles") {
        strncpy_s(_telemetry->config.map.tiles, value.c_str(), URL_SIZE);
      } else if (name == "map_api_key") {
        strncpy_s(_telemetry->config.map.apiKey, value.c_str(), API_KEY_SIZE);
      } else if (name == "mqtt_broker") {
        strncpy_s(_telemetry->config.mqtt.broker, value.c_str(), ENDPOINT_SIZE);
      } else if (name == "mqtt_port") {
        _telemetry->config.mqtt.port = atoi(value.c_str());
      } else if (name == "mqtt_topic") {
        strncpy_s(_telemetry->config.mqtt.topic, value.c_str(), TOPIC_SIZE);
      } else if (name == "internal_sensors") {
        _telemetry->config.internalSensors.enableHallEffect = (value == "on");
      }
    }
    _telemetry->save();
    _status = savedStatus;
  }
  request->send(SPIFFS, "/settings.html", "text/html", false, settingsTemplateProcessor);
}

bool webEmitSensor(const Sensor& sensor) {
  static uint32_t lastEmit = 0;
  if (isWebRunning && webSocket->count() > 0 && (millis() - lastEmit) > WS_EMIT_RATE) {
    char minJson[JSON_BUFFER_SIZE];
    int len = writeSensorJson(minJson, sensor, false);
    if (len > JSON_BUFFER_SIZE) {
      LOGE("WS buffer size exceeded! (%d)", len);
    }
    int16_t idx = sensor._index;
    bool dataSent = false;
    for (AsyncWebSocketClient* c : webSocket->getClients()) {
      if (c->_tempObject && !c->queueIsFull()) {
        if (((bool*)c->_tempObject)[idx]) {
          c->text(minJson);
          dataSent = true;
        } else {
          char fullJson[JSON_BUFFER_SIZE];
          int len = writeSensorJson(fullJson, sensor, true);
          if (len > JSON_BUFFER_SIZE) {
            LOGE("WS buffer size exceeded! (%d)", len);
          }
          c->text(fullJson);
          dataSent = true;
          ((bool*)c->_tempObject)[idx] = true;
        }
      }
    }
    lastEmit = millis();
    return dataSent;
  } else {
    return false;
  }
}

int writeSensorJson(char* out, const Sensor& sensor, bool all) {
  char value[JSON_VALUE_BUFFER_SIZE];
  int len = jsonWriteSensorValue(value, sensor);
  if (len > JSON_VALUE_BUFFER_SIZE-1) {
    LOGE("Value of sensor %04X exceeded buffer size!", sensor.sensorId);
  }
  const char *name;
  const char *unit;
  if (sensor.info) {
    name = sensor.info->name;
    unit = sensor.info->getUnitName();
    if (sensor.sensorId == FLIGHT_MODE_ID) {
      unit = "";
    } else if (sensor.sensorId == GPS_STATE_ID) {
      unit = "";
    }
  } else {
    // unknown sensor
    name = "Custom";
    unit = "";
  }
  if (all) {
    return sprintf(out, "{\"physicalId\": \"%02X\", \"sensorId\": \"%04X\", \"name\": \"%s\", \"value\": %s, \"unit\": \"%s\"}", sensor.physicalId, sensor.sensorId, name, value, unit);
  } else {
    return sprintf(out, "{\"physicalId\": \"%02X\", \"sensorId\": \"%04X\", \"value\": %s}", sensor.physicalId, sensor.sensorId, value);
  }
}
