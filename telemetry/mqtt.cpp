#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <SPIFFS.h>

#include "protocol.h"
#include "mqtt.h"
#include "json.h"
#include "debug.h"

#define TOPIC_BUFFER_SIZE 64
#define PAYLOAD_BUFFER_SIZE 128
#define RECONNECT_DELAY 372

static Telemetry* _telemetry;

static WiFiClientSecure* client = nullptr;
static MQTTClient* mqttClient = nullptr;
static char* caCert = nullptr;
static char* cert = nullptr;
static char* privKey = nullptr;

static bool isMqttRunning = false;

char* loadFromFile(const char* fname) {
  char* buf = nullptr;
  if (SPIFFS.exists(fname)) {
    File f = SPIFFS.open(fname);
    const size_t size = f.size();
    buf = new char[size+1];
    if (size == f.readBytes(buf, size)) {
      buf[size] = '\0';
    } else {
      delete[] buf;
      buf = nullptr;
    }
    LOGD("Loaded file: %d (name %s, size %d)", buf != nullptr, f.name(), size);
    f.close();
  }
  return buf;
}

void loadCertificates(WiFiClientSecure*const client) {
  LOGMEM("pre-MQTT-loadCertificates");
  if (!caCert) {
    caCert = loadFromFile("/ca.cert.pem");
    client->setCACert(caCert);
  }
  if (!cert) {
    cert = loadFromFile("/client.cert.pem");
    client->setCertificate(cert);
  }
  if (!privKey) {
    privKey = loadFromFile("/private.key.pem");
    client->setPrivateKey(privKey);
  }
  LOGMEM("post-MQTT-loadCertificates");
}

void freeCertificates(WiFiClientSecure*const client) {
  LOGMEM("pre-MQTT-freeCertificates");
  if (caCert) {
    client->setCACert(nullptr);
    delete[] caCert;
    caCert = nullptr;
  }
  if (cert) {
    client->setCertificate(nullptr);
    delete[] cert;
    cert = nullptr;
  }
  if (privKey) {
    client->setPrivateKey(nullptr);
    delete[] privKey;
    privKey = nullptr;
  }
  LOGMEM("post-MQTT-freeCertificates");
}

void mqttBegin(Telemetry* telemetry) {
  if (!isMqttRunning && strlen(telemetry->config.mqtt.broker) > 0) {
    _telemetry = telemetry;
    client = new WiFiClientSecure();
    mqttClient = new MQTTClient(256);
    mqttClient->begin(_telemetry->config.mqtt.broker, _telemetry->config.mqtt.port, *client);
    isMqttRunning = true;
  }
}

void mqttStop() {
  if (isMqttRunning) {
    mqttClient->disconnect();
    if (mqttClient) {
      delete mqttClient;
      mqttClient = nullptr;
    }
    if (client) {
      delete client;
      client = nullptr;
    }
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
  static uint32_t lastAttempt = 0;
  bool isConn = mqttClient->connected();
  const uint32_t ms = millis();
  if (!isConn && (ms - lastAttempt) > RECONNECT_DELAY) {
    LOGMEM("pre-MQTT-ensureConnected");
    LOGD("WiFi station status: %d", WiFi.status());
    loadCertificates(client);
    isConn = mqttClient->connect(_telemetry->config.wifi.client.hostname);
    LOGD("MQTT connection: %d", mqttClient->returnCode());
    if (isConn) {
      lastAttempt = 0;
      freeCertificates(client);
    } else {
      lastAttempt = ms;
      LOGE("MQTT connection error: %d", mqttClient->lastError());
      char errMsg[100] = {'\0'};
      int errCode = client->lastError(errMsg, 100);
      LOGE("MQTT client connection error (%d): %s", errCode, errMsg);
    }
  }
  return isConn;
}

bool mqttPublishSensor(const Sensor& sensor) {
  if (isMqttRunning && WiFi.isConnected()) {
    if (ensureConnected()) {
      char* payload = new char[PAYLOAD_BUFFER_SIZE];
      int payloadPos = sprints(payload, "{\"value\": ");
      int len = protocolWriteJsonSensorValue(payload+payloadPos, sensor);
      if (len < 0) {
        LOGE("protocolWriteJsonSensorValue error: %d", len);
        delete[] payload;
        return false;
      } else if (len > JSON_VALUE_BUFFER_SIZE-1) {
        LOGE("Value of sensor %04X exceeded buffer size!", sensor.sensorId);
        delete[] payload;
        return false;
      }
      payloadPos += len;

      const char *name;
      const char *unit;
      int8_t slot;
      if (sensor.info) {
        name = sensor.info->name;
        unit = sensor.info->getUnitName();
        if (sensor.info->lastId > sensor.info->firstId) {
          slot = sensor.sensorId - sensor.info->firstId + 1;
        } else {
          slot = -1;
        }
      } else {
        // unknown sensor
        name = UNKNOWN_SENSOR;
        unit = "";
        slot = -1;
      }

      char* topicBuf = new char[TOPIC_BUFFER_SIZE];
      len = sprintf(topicBuf, "%s/%02X/%s", _telemetry->config.mqtt.topic, sensor.physicalId, name);
      if (sensor.info->lastId > sensor.info->firstId) {
        // Append slot and subId if sensor name is not unique
        if (len > 0 && len < TOPIC_BUFFER_SIZE && slot > 0) {
          int slotLen = sprintf(topicBuf+len, "/%d", slot);
          len = (slotLen >=0 ? len+slotLen : -1);
        }
        if (len > 0 && len < TOPIC_BUFFER_SIZE && sensor.info && sensor.info->subCount > 1) {
          int subLen;
          if (slot > 0) {
            subLen = sprintf(topicBuf+len, "/%d", sensor.info->subId+1);
          } else {
            subLen = sprintf(topicBuf+len, "//%d", sensor.info->subId+1);
          }
          len = (subLen >=0 ? len+subLen : -1);
        }
        if (len < 0) {
          LOGE("sprintf error: %d", len);
          delete[] topicBuf;
          delete[] payload;
          return false;
        } else if (len > TOPIC_BUFFER_SIZE-1) {
          LOGE("Topic buffer size exceeded! (%d)", len);
          delete[] topicBuf;
          delete[] payload;
          return false;
        }
      }
      // remove any leading '/'
      char* topic = (topicBuf[0] == '/') ? topicBuf+1 : topicBuf;

      if (strlen(unit) > 0) {
        len = sprints(payload+payloadPos, ", \"unit\": \"");
        payloadPos += len;
        len = sprints(payload+payloadPos, unit);
        payloadPos += len;
        payload[payloadPos++] = '\"';
      }
      payload[payloadPos++] = '}';
      payload[payloadPos++] = '\0';
      bool rc;
      if (payloadPos <= PAYLOAD_BUFFER_SIZE) {
        rc = mqttClient->publish(topic, payload);
      } else {
        LOGE("Payload buffer size exceeded! (%d)", payloadPos);
        rc = false;
      }
      delete[] topicBuf;
      delete[] payload;
      return rc;
    }
  }
  return false;
}
