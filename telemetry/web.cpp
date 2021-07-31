#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#include "protocol.h"
#include "web.h"
#include "json.h"
#include "debug.h"

#define SENSOR_ID_BUFFER_SIZE 8
#define JSON_BUFFER_SIZE 256
#define WS_EMIT_RATE 100

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
    webServer->begin();
    isWebRunning = true;
  }
  LOGMEM("post-webBegin");
}

void webStop() {
  if (isWebRunning) {
    webServer->end();
    _telemetry = nullptr;
    isWebRunning = false;
  }
  /*
   * currently causes CORRUPT HEAP: Bad head
  if (webServer) {
    delete webServer;
    webServer = nullptr;
  }
  if (webSocket) {
    delete webSocket;
    webSocket = nullptr;
  }
  */
  LOGMEM("post-webStop");
}

void wsEventHandler(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  bool* haveSentFull;
  switch (type) {
    case WS_EVT_CONNECT:
      LOGD("WS connected");
      haveSentFull = new bool[MAX_SENSORS];
      memset(haveSentFull, 0, MAX_SENSORS);
      client->_tempObject = haveSentFull;
      break;
    case WS_EVT_DISCONNECT:
      LOGD("WS disconnected");
      if (client->_tempObject) {
        delete[] (bool*) (client->_tempObject);
        client->_tempObject = nullptr;
      }
      break;
    default:
      ;
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

void webLoop(uint32_t ms) {
  static uint32_t lastCleanup = 0;
  if (isWebRunning && (ms - lastCleanup) > 500) {
    webSocket->cleanupClients();
    lastCleanup = ms;
  }
}

void sendJson(AsyncWebServerRequest* request) {
  AsyncResponseStream* response = request->beginResponseStream("application/json");
  response->print("{\"sensors\": [\n");
  char* json = new char[JSON_BUFFER_SIZE];
  bool isFirst = true;
  for (int i=0; i<_telemetry->numSensors; i++) {
    int len = writeSensorJson(json, _telemetry->sensors[i], true);
    if (len < 0) {
      LOGE("writeSensorJson error: %d", len);
      delete[] json;
      return;
    } else if (len > JSON_BUFFER_SIZE) {
      LOGE("JSON buffer size exceeded! (%d)", len);
      delete[] json;
      return;
    }
    if (isFirst) {
      isFirst = false;
    } else {
      response->print(",\n");
    }
    response->print(json);
  }
  response->print("\n]}");
  delete[] json;
  request->send(response);
}

static String _status = empty;

const String settingsTemplateProcessor(const String& var) {
  static const String checked("checked");
  if (var == "SOURCE_UART") {
    return _telemetry->config.input.source == SOURCE_UART ? checked : empty;
  } else if (var == "SOURCE_BLE") {
    return _telemetry->config.input.source == SOURCE_BLE ? checked : empty;
  } else if (var == "BT_SOURCE") {
    return _telemetry->config.input.btAddress;
  } else if (var == "PROTOCOL_SPORT") {
    return _telemetry->config.input.protocol == PROTOCOL_SMART_PORT ? checked : empty;
  } else if (var == "PROTOCOL_CRSF") {
    return _telemetry->config.input.protocol == PROTOCOL_CRSF ? checked : empty;
  } else if (var == "USB_DISABLED") {
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
  } else if (var == "WIFI_OFF") {
    return _telemetry->config.wifi.mode == WIFI_OFF ? checked : empty;
  } else if (var == "WIFI_STA") {
    return _telemetry->config.wifi.mode == WIFI_STA ? checked : empty;
  } else if (var == "WIFI_AP") {
    return _telemetry->config.wifi.mode == WIFI_AP ? checked : empty;
  } else if (var == "WIFI_AP_STA") {
    return _telemetry->config.wifi.mode == WIFI_AP_STA ? checked : empty;
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
      if (name == "source") {
        if (value == "ble") {
          _telemetry->config.input.source = SOURCE_BLE;
        } else {
          _telemetry->config.input.source = SOURCE_UART;
        }
      } else if (name == "bt_source") {
        strncpy_s(_telemetry->config.input.btAddress, value.c_str(), BD_ADDR_SIZE);
      } else if (name == "protocol") {
        if (value == "crsf") {
          _telemetry->config.input.protocol = PROTOCOL_CRSF;
        } else {
          _telemetry->config.input.protocol = PROTOCOL_SMART_PORT;
        }
      } else if (name == "usb_mode") {
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
      } else if (name == "wifi_mode") {
        if (value == "sta") {
          _telemetry->config.wifi.mode = WIFI_STA;
        } else if (value == "ap") {
          _telemetry->config.wifi.mode = WIFI_AP;
        } else if (value == "ap_sta") {
          _telemetry->config.wifi.mode = WIFI_AP_STA;
        } else {
          _telemetry->config.wifi.mode = WIFI_OFF;
        }
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
    char* minJson = new char[JSON_BUFFER_SIZE];
    int len = writeSensorJson(minJson, sensor, false);
    if (len < 0) {
      LOGE("writeSensorJson error: %d", len);
      delete[] minJson;
      return false;
    } else if (len > JSON_BUFFER_SIZE) {
      LOGE("WS buffer size exceeded! (%d)", len);
      delete[] minJson;
      return false;
    }
    int16_t idx = sensor._index;
    bool dataSent = false;
    char *fullJson = nullptr;
    for (AsyncWebSocketClient* c : webSocket->getClients()) {
      bool* haveSentFull = (bool*) (c->_tempObject);
      if (haveSentFull && !c->queueIsFull()) {
        if (haveSentFull[idx]) {
          c->text(minJson);
          dataSent = true;
        } else {
          if (!fullJson) {
            fullJson = new char[JSON_BUFFER_SIZE];
            int len = writeSensorJson(fullJson, sensor, true);
            if (len < 0) {
              LOGE("writeSensorJson error: %d", len);
              delete[] minJson;
              if (fullJson) {
                delete[] fullJson;
              }
              return false;
            } else if (len > JSON_BUFFER_SIZE) {
              LOGE("WS buffer size exceeded! (%d)", len);
              delete[] minJson;
              if (fullJson) {
                delete[] fullJson;
              }
              return false;
            }
          }
          c->text(fullJson);
          dataSent = true;
          haveSentFull[idx] = true;
        }
      }
    }
    delete[] minJson;
    if (fullJson) {
        delete[] fullJson;
    }
    lastEmit = millis();
    return dataSent;
  } else {
    return false;
  }
}

/**
 * Returns -1 on error.
 */
int writeSensorJson(char* out, const Sensor& sensor, bool all) {
  int len;
  char sensorId[SENSOR_ID_BUFFER_SIZE];
  if (sensor.info && sensor.info->subCount > 1) {
    len = sprintf(sensorId, "%04X/%d", sensor.sensorId, sensor.info->subId);
  } else {
    len = sprintf(sensorId, "%04X", sensor.sensorId);
  }
  if (len < 0) {
    LOGE("sprintf error: %d", len);
    return -1;
  } else if (len > SENSOR_ID_BUFFER_SIZE-1) {
    LOGE("Sensor ID %04X exceeded buffer size!", sensor.sensorId);
    return -1;
  }

  char* value = new char[JSON_VALUE_BUFFER_SIZE];
  len = protocolWriteJsonSensorValue(value, sensor);
  if (len < 0) {
    LOGE("protocolWriteJsonSensorValue error: %d", len);
    delete[] value;
    return -1;
  } else if (len > JSON_VALUE_BUFFER_SIZE-1) {
    LOGE("Value of sensor %04X exceeded buffer size!", sensor.sensorId);
    delete[] value;
    return -1;
  }

  if (all) {
    const char *name;
    const char *unit;
    if (sensor.info) {
      name = sensor.info->name;
      unit = sensor.info->getUnitName();
    } else {
      // unknown sensor
      name = UNKNOWN_SENSOR;
      unit = "";
    }
    len = sprintf(out, "{\"physicalId\": \"%02X\", \"sensorId\": \"%s\", \"name\": \"%s\", \"value\": %s, \"unit\": \"%s\"}", sensor.physicalId, sensorId, name, value, unit);
  } else {
    len = sprintf(out, "{\"physicalId\": \"%02X\", \"sensorId\": \"%s\", \"value\": %s}", sensor.physicalId, sensorId, value);
  }
  delete[] value;
  return len;
}
