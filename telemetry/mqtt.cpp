#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <SPIFFS.h>

#include "json.h"
#include "mqtt.h"
#include "debug.h"

#define TOPIC_BUFFER_SIZE 64
#define PAYLOAD_BUFFER_SIZE 128

static Telemetry* _telemetry;

static WiFiClientSecure* client = nullptr;
static MQTTClient* mqttClient = nullptr;

static bool isMqttRunning = false;

template<typename L> void loadFromFile(const char* fname, L&& load) {
  if (SPIFFS.exists(fname)) {
    File f = SPIFFS.open(fname);
    bool rc = load(f, f.size());
    LOGD("Loaded: %d (name %s, size %d)", rc, f.name(), f.size());
    f.close();
  }
}

void loadCertificates(WiFiClientSecure* client) {
  LOGMEM();
  SPIFFS.begin();
  loadFromFile("/ca.cert.pem", [client](Stream& stream, size_t size){return client->loadCACert(stream, size);});
  loadFromFile("/client.cert.pem", [client](Stream& stream, size_t size){return client->loadCertificate(stream, size);});
  loadFromFile("/private.key.pem", [client](Stream& stream, size_t size){return client->loadPrivateKey(stream, size);});
  SPIFFS.end();
  LOGMEM();
}

void mqttBegin(Telemetry* telemetry) {
  if (!isMqttRunning && telemetry->config.mqtt.broker) {
    _telemetry = telemetry;
    client = new WiFiClientSecure();
    loadCertificates(client);
    mqttClient = new MQTTClient(256);
    mqttClient->begin(_telemetry->config.mqtt.broker, _telemetry->config.mqtt.port, *client);
    isMqttRunning = true;
  }
}

void mqttStop() {
  if (isMqttRunning) {
    mqttClient->disconnect();
    delete mqttClient;
    delete client;
    _telemetry = nullptr;
    isMqttRunning = false;
  }
}

void mqttLoop() {
  if (isMqttRunning) {
    mqttClient->loop();
  }
}

bool ensureConnected() {
  LOGMEM();
  bool isConn = mqttClient->connected();
  if (!isConn) {
    LOGD("WiFi station status: %d", WiFi.status());
    isConn = mqttClient->connect(_telemetry->config.wifi.client.hostname);
    LOGD("MQTT connection: %d", mqttClient->returnCode());
    if (!isConn) {
      LOGE("MQTT connection error: %d", mqttClient->lastError());
      char errMsg[100] = {'\0'};
      int errCode = client->lastError(errMsg, 100);
      LOGE("Client connection error (%d): %s", errCode, errMsg);
    }
  }
  return isConn;
}

bool mqttPublishSensor(const Sensor& sensor) {
  if (isMqttRunning && WiFi.isConnected()) {
    if (ensureConnected()) {
      char value[JSON_VALUE_BUFFER_SIZE];
      int len = jsonWriteSensorValue(value, sensor);
      if (len > JSON_VALUE_BUFFER_SIZE-1) {
        LOGE("Value of sensor %04X exceeded buffer size!", sensor.sensorId);
      }
      const char *name;
      const char *unit;
      int8_t pos;
      if (sensor.info) {
        name = sensor.info->name;
        unit = sensor.info->getUnitName();
        if (sensor.info->lastId > sensor.info->firstId) {
          pos = sensor.sensorId - sensor.info->firstId + 1;
          if (sensor.sensorId == FLIGHT_MODE_ID) {
            unit = "";
            pos = -1;
          } else if (sensor.sensorId == GPS_STATE_ID) {
            unit = "";
            pos = -1;
          }
        } else {
          pos = -1;
        }
      } else {
        // unknown sensor
        name = "Custom";
        unit = "";
        pos = -1;
      }
      char topicBuf[TOPIC_BUFFER_SIZE];
      if (pos > 0) {
        len = sprintf(topicBuf, "%s/%02X/%s/%d", _telemetry->config.mqtt.topic, sensor.physicalId, name, pos);
      } else {
        len = sprintf(topicBuf, "%s/%02X/%s", _telemetry->config.mqtt.topic, sensor.physicalId, name);
      }
      if (len > TOPIC_BUFFER_SIZE-1) {
        LOGE("Topic buffer size exceeded! (%d)", len);
      }
      // remove any leading '/'
      char* topic = (topicBuf[0] == '/') ? topicBuf+1 : topicBuf;
      char payload[PAYLOAD_BUFFER_SIZE];
      if (strlen(unit) > 0) {
        len = sprintf(payload, "{\"value\": %s, \"unit\": \"%s\"}", value, unit);
      } else {
        len = sprintf(payload, "{\"value\": %s}", value);
      }
      if (len > PAYLOAD_BUFFER_SIZE-1) {
        LOGE("Payload buffer size exceeded! (%d)", len);
      }
      return mqttClient->publish(topic, payload);
    }
  }
  return false;
}
