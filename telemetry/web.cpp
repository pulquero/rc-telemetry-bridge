#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "web.h"
#include "debug.h"

#define JSON_BUFFER_SIZE 256
#define JSON_VALUE_BUFFER_SIZE 128
#define WS_EMIT_RATE 300

static void wsEventHandler(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
static void sendJson(AsyncWebServerRequest* request);
static void sendSettings(AsyncWebServerRequest* request);
static int writeSensorJson(char* out, const Sensor& sensor, bool all);
static int writeFlightModeArray(char* out, uint32_t modes);
static int writeGpsStateArray(char* out, uint32_t modes);
static const String indexTemplateProcessor(const String& var);
static const String settingsTemplateProcessor(const String& var);

static Telemetry* _telemetry;

static const String empty = String();
static AsyncWebServer webServer(80);
static AsyncWebSocket webSocket("/ws");
static bool isWebRunning = false;

void webBegin(Telemetry* telemetry) {
  if (!isWebRunning) {
    _telemetry = telemetry;
    SPIFFS.begin();
    webSocket.onEvent(wsEventHandler);
    // first match wins
    webServer.addHandler(&webSocket);
    webServer.on("/sensors", sendJson);
    webServer.on("/settings.html", sendSettings);
    webServer.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html").setTemplateProcessor(indexTemplateProcessor);
    webServer.begin();
    isWebRunning = true;
  }
}

void webStop() {
  if (isWebRunning) {
    webServer.end();
    SPIFFS.end();
    _telemetry = nullptr;
    isWebRunning = false;
  }
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
  if ((millis() - lastCleanup) > 500) {
    webSocket.cleanupClients();
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
      LOGE("JSON buffer exceeded!");
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
  static const String checked = String("checked");
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
  } else if (var == "INTERNAL_SENSORS") {
    return _telemetry->config.internalSensors.enableHallEffect ? checked : empty;
  } else if (var == "STATUS") {
    return _status;
  } else {
    return empty;
  }
}

void sendSettings(AsyncWebServerRequest* request) {
  static const String savedStatus = String("Saved");
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
  if (webSocket.count() > 0 && (millis() - lastEmit) > WS_EMIT_RATE) {
    char minJson[JSON_BUFFER_SIZE];
    int len = writeSensorJson(minJson, sensor, false);
    if (len > JSON_BUFFER_SIZE) {
      LOGE("WS buffer exceeded!");
    }
    int16_t idx = sensor._index;
    bool dataSent = false;
    for (AsyncWebSocketClient* c : webSocket.getClients()) {
      if (c->_tempObject && !c->queueIsFull()) {
        if (((bool*)c->_tempObject)[idx]) {
          c->text(minJson);
          dataSent = true;
        } else {
          char fullJson[JSON_BUFFER_SIZE];
          int len = writeSensorJson(fullJson, sensor, true);
          if (len > JSON_BUFFER_SIZE) {
            LOGE("WS buffer exceeded!");
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
  const char *name;
  const char *unit;
  char value[JSON_VALUE_BUFFER_SIZE];
  if (sensor.info) {
    name = sensor.info->name;
    unit = sensor.info->getUnitName();
    if (sensor.sensorId == FLIGHT_MODE_ID) {
      int len = writeFlightModeArray(value, sensor.value.numeric);
      if (len > JSON_VALUE_BUFFER_SIZE) {
        LOGE("Flight mode array exceeded value buffer!");
      }
      unit = "";
    } else if (sensor.sensorId == GPS_STATE_ID) {
      int len = writeGpsStateArray(value, sensor.value.numeric);
      if (len > JSON_VALUE_BUFFER_SIZE) {
        LOGE("GPS state array exceeded value buffer!");
      }
      unit = "";
    } else if (sensor.info->unit == UNIT_GPS) {
      char lon[16], lat[16];
      dtostrf(sensor.value.gps.longitude/10000.0/60.0, 9, 7, lon);
      dtostrf(sensor.value.gps.latitude/10000.0/60.0, 9, 7, lat);
      sprintf(value, "[%s, %s]", lon, lat);
    } else {
      if (sensor.info->precision) {
        double v = sensor.value.numeric;
        for (int i=0; i<sensor.info->precision; i++) {
          v /= 10.0;
        }
        dtostrf(v, 2+sensor.info->precision, sensor.info->precision, value);
      } else {
        itoa(sensor.value.numeric, value, 10);
      }
    }
  } else {
    // unknown sensor
    name = "";
    unit = "";
    itoa(sensor.value.numeric, value, 10);
  }
  if (all) {
    return sprintf(out, "{\"physicalId\": \"%02X\", \"sensorId\": \"%04X\", \"name\": \"%s\", \"value\": %s, \"unit\": \"%s\"}", sensor.physicalId, sensor.sensorId, name, value, unit);
  } else {
    return sprintf(out, "{\"physicalId\": \"%02X\", \"sensorId\": \"%04X\", \"value\": %s}", sensor.physicalId, sensor.sensorId, value);
  }
}

int writeFlightModeArray(char* out, uint32_t modes) {
  int pos = 0;
  out[pos++] = '[';
  uint32_t v = modes;
  uint8_t flag = v % 10;
  if (flag == 1) {
    pos += sprintf(out+pos, "\"DISARMED\",");
  } else if(flag == 2) {
    pos += sprintf(out+pos, "\"CANT_ARM\",");
  } else if(flag == 5) {
    pos += sprintf(out+pos, "\"ARMED\",");
  }
  
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprintf(out+pos, "\"ANGLE\",");
  }
  if((flag & 2) == 2) {
    pos += sprintf(out+pos, "\"HORIZON\",");
  }
  if((flag & 4) == 4) {
    pos += sprintf(out+pos, "\"MANUAL\",");
  }
  
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprintf(out+pos, "\"HEADING\",");
  }
  if((flag & 2) == 2) {
    pos += sprintf(out+pos, "\"NAV_ALTHOLD\",");
  }
  if((flag & 4) == 4) {
    pos += sprintf(out+pos, "\"NAV_POSHOLD\",");
  }
  
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprintf(out+pos, "\"NAV_RTH\",");
  }
  if ((flag & 8) == 8) {
    pos += sprintf(out+pos, "\"NAV_CRUISE\",");
  } else if ((flag & 2) == 2) {
    pos += sprintf(out+pos, "\"NAV_WP\",");
  } else if ((flag & 4) == 4) {
    pos += sprintf(out+pos, "\"HEADFREE\",");
  }

  out[pos-1] = ']';
  return pos;
}

int writeGpsStateArray(char* out, uint32_t state) {
  char* fix = "WAIT";
  uint8_t flag = state/1000 % 10;
  if (flag == 1) {
    fix = "FIX";
  } else if (flag == 3) {
    fix = "HOME";
  } else if (flag == 7) {
    fix = "HOME RESET";
  }
  return sprintf(out, "[%d, %d, \"%s\"]", state % 100, (state/100) % 10, fix);
}
