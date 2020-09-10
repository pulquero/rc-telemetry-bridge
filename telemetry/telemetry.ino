// No OTA (2MB APP/2MB SPIFFS)

#include <Arduino.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Preferences.h>
#include "protocol.h"
#include "telemetry.h"
#include "web.h"
#include "mqtt.h"

#include "debug.h"

#define SERIALIZED_PACKET_SIZE (1+SPORT_DATA_PACKET_LEN+2+1) // allow for two byte stuffings
#define SERIALIZATION_BUFFER_SIZE (1+2*SPORT_DATA_PACKET_LEN+1) // allow for everything to be byte-stuffed
#define SERVICE_UUID        "0000FFF0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFF6-0000-1000-8000-00805F9B34FB"
#define BLE_BUFFER_SIZE (2*SERIALIZED_PACKET_SIZE)
#define MTU_OVERHEAD 3
#define PHYSICAL_SENSOR_ID 0x19
#define TOUCH_THRESHOLD 35
#define MIN_TOUCH_TIME 307
#define TOUCH_RESET_TIME 1013
#define RX_PIN 16
#define WIFI_STA_TOUCH_PIN 13
#define WIFI_AP_TOUCH_PIN 14
#define BLE_ADVERT_TOUCH_PIN 15
#define WIFI_STA_LED_PIN 19 // green
#define WIFI_AP_LED_PIN 18 // yellow
#define BLE_ADVERT_LED_PIN 21 // blue
#define PACKET_LED_PIN 32 // red
#define WEB_LED_PIN 33 // white

static const IPAddress localHost(192, 168, 31, 1);
static const IPAddress gateway(192, 168, 31, 1);
static const IPAddress subnet(255, 255, 255, 0);

static BluetoothSerial* btSerial;
static bool btCongested = false;
static BLEServer* bleServer;
static BLECharacteristic* bleChar;
static bool bleIsAdvertising = false;
static uint16_t bleDataSize = 23 - MTU_OVERHEAD;

static uint32_t serialPacketCount = 0;
static uint32_t webMsgCount = 0;

static const uint8_t touchPins[] = {WIFI_STA_TOUCH_PIN, BLE_ADVERT_TOUCH_PIN, WIFI_AP_TOUCH_PIN};
static Telemetry telemetry;
#define INCOMING_CAPACITY 20
static uint8_t incoming[INCOMING_CAPACITY];
static uint8_t incomingReadPos = 0;
static uint8_t incomingWritePos = 0;

static void checkButtons();
static bool handleButton(uint8_t buttonId);
static bool startWiFiAP();
static bool startWiFiSTA();
static bool stopWiFi();
static void sendInternalSensors(bool includeStart);
static void loadPreferenceString(Preferences& prefs, const char* key, char* value, int maxSize, const char* defaultValue);

void btTelemetryCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  switch(event) {
    case ESP_SPP_SRV_OPEN_EVT:
      LOGD("SPP opened");
      btCongested = false;
      break;
    case ESP_SPP_CLOSE_EVT:
      LOGD("SPP closed");
      break;
    case ESP_SPP_CONG_EVT:
      LOGD("SPP congestion %d", param->cong.cong);
      btCongested = param->cong.cong;
      break;
  }
}

void bleTelemetryCallback(esp_gatts_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gatts_cb_param_t* param) {
  switch(event) {
    case ESP_GATTS_MTU_EVT:
      LOGD("BLE MTU set to %d", param->mtu.mtu);
      // ensure we don't exceed the buffer size
      bleDataSize = min(param->mtu.mtu - MTU_OVERHEAD, BLE_BUFFER_SIZE);
      break;
  }
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  telemetry.load();

// Bluetooth startup sequence is critical!
  if (telemetry.config.ble.mode != MODE_DISABLED) {
    BLEDevice::init(telemetry.config.ble.name);
    BLEDevice::setMTU(BLE_BUFFER_SIZE+MTU_OVERHEAD); // max 517
    BLEDevice::setPower(ESP_PWR_LVL_P7);
    BLEDevice::setCustomGattsHandler(bleTelemetryCallback);
  }
  if (telemetry.config.bt.mode != MODE_DISABLED) {
    btSerial = new BluetoothSerial();
    btSerial->register_callback(btTelemetryCallback);
    if(!btSerial->begin(telemetry.config.bt.name)) {
      LOGE("Failed to initialise BT");
    }
  }
  if (telemetry.config.ble.mode != MODE_DISABLED) {
    bleServer = BLEDevice::createServer();
    BLEService* bleService = bleServer->createService(SERVICE_UUID);
    bleChar = bleService->createCharacteristic(CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
    BLE2902* ble2902Desc = new BLE2902();
    ble2902Desc->setNotifications(true);
    bleChar->addDescriptor(ble2902Desc);
    bleService->start();
    BLEAdvertising* bleAdvertising = BLEDevice::getAdvertising();
    bleAdvertising->addServiceUUID(SERVICE_UUID);
    bleAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
  }

  if ((telemetry.config.wifi.mode & WIFI_AP)) {
    startWiFiAP();
  }
  if ((telemetry.config.wifi.mode & WIFI_STA)) {
    startWiFiSTA();
  }

#ifndef DEBUG
  if (telemetry.config.usb.mode != MODE_DISABLED) {
    Serial.begin(57600);
  }
#endif
  Serial2.begin(57600, SERIAL_8N1, RX_PIN, -1, true);

  pinMode(WIFI_STA_LED_PIN, OUTPUT);
  pinMode(WIFI_AP_LED_PIN, OUTPUT);
  pinMode(BLE_ADVERT_LED_PIN, OUTPUT);
  pinMode(WEB_LED_PIN, OUTPUT);
  pinMode(PACKET_LED_PIN, OUTPUT);

  LOGMEM();
}

void write(uint8_t* data, uint8_t len, SerialMode mode) {
#ifndef DEBUG
  if (telemetry.config.usb.mode == mode) {
    if (mode == MODE_FILTER) {
      // skip initial START_STOP
      data++;
      len--;
    }
    Serial.write(data, len);
  }
#endif
  if (telemetry.config.bt.mode == mode && !btCongested) {
    if (mode == MODE_FILTER) {
      // skip initial START_STOP
      data++;
      len--;
    }
    btSerial->write(data, len);
  }
  if (telemetry.config.ble.mode == mode) {
    static uint8_t bleBuffer[BLE_BUFFER_SIZE];
    static uint8_t bleWritePos = 0;
    if (mode == MODE_FILTER) {
      // if it doesn't fit and there is already data in the buffer then send it
      if (len > bleDataSize - bleWritePos && bleWritePos > 0) {
        bleChar->setValue(bleBuffer, bleWritePos);
        bleChar->notify();
        bleWritePos = 0;
      }
      if (len <= bleDataSize - bleWritePos) {
        for (int i=0; i<len; i++) {
          bleBuffer[bleWritePos++] = data[i];
        }
      } else {
        LOGE("Discarding BLE data - exceeds MTU");
      }
      if (bleWritePos > bleDataSize - SERIALIZED_PACKET_SIZE) {
        bleChar->setValue(bleBuffer, bleWritePos);
        bleChar->notify();
        bleWritePos = 0;
      }
    } else {
      for (int i=0; i<len; i++) {
        bleBuffer[bleWritePos++] = data[i];
        if (bleWritePos == bleDataSize) {
          bleChar->setValue(bleBuffer, bleWritePos);
          bleChar->notify();
          bleWritePos = 0;
        }
      }
    }
  }
}

void loop() {
  static uint32_t lastSerialCheckpoint = 0;
  if (incomingReadPos < incomingWritePos) {
    uint8_t* nextByte = incoming+incomingReadPos++;
    write(nextByte, 1, MODE_PASS_THRU);
    sportOnReceive(*nextByte);
    if (incomingReadPos == incomingWritePos) {
      incomingReadPos = 0;
      incomingWritePos = 0;
    }
  } else {
    // rx is floating by default so when nothing is connected we may read noise
    if (Serial2.available()) {
      static uint8_t buf[1];
      buf[0] = Serial2.read();
      write(buf, 1, MODE_PASS_THRU);
      if (sportOnReceive(buf[0]) > 0) {
        serialPacketCount++;
      }
    }
    if ((millis() - lastSerialCheckpoint) > 97) {
      if (serialPacketCount == 0) {
        // received nothing or garbage so resort to sending our own sensors
        sendInternalSensors(true);
      }
      serialPacketCount = 0;
      lastSerialCheckpoint = millis();
    }
  }

  checkButtons();

  uint32_t ledCounter = millis()%400;
  if (serialPacketCount > 0 && ledCounter > 100 && ledCounter < 200) {
    static uint32_t lastSerialPacketCount = 0;
    if (serialPacketCount != lastSerialPacketCount) {
      digitalWrite(PACKET_LED_PIN, HIGH);
      lastSerialPacketCount = serialPacketCount;
    }
  } else {
    digitalWrite(PACKET_LED_PIN, LOW);
  }
  if (webMsgCount > 0 && ledCounter > 200 && ledCounter < 300) {
    static uint32_t lastWebMsgCount = 0;
    if (webMsgCount != lastWebMsgCount) {
      digitalWrite(WEB_LED_PIN, HIGH);
      lastWebMsgCount = webMsgCount;
    }
  } else {
    digitalWrite(WEB_LED_PIN, LOW);
  }
  if (bleIsAdvertising) {
    digitalWrite(BLE_ADVERT_LED_PIN, ledCounter > 300 ? HIGH : LOW);
  } else {
    digitalWrite(BLE_ADVERT_LED_PIN, LOW);
  }
  switch (WiFi.getMode()) {
    case WIFI_AP:
      digitalWrite(WIFI_AP_LED_PIN, ledCounter < 100 ? HIGH : LOW);
      digitalWrite(WIFI_STA_LED_PIN, LOW);
      break;
    case WIFI_STA:
      digitalWrite(WIFI_AP_LED_PIN, LOW);
      digitalWrite(WIFI_STA_LED_PIN, ledCounter < 100 ? HIGH : LOW);
      break;
    case WIFI_AP_STA:
      digitalWrite(WIFI_AP_LED_PIN, ledCounter < 100 ? HIGH : LOW);
      digitalWrite(WIFI_STA_LED_PIN, ledCounter < 100 ? HIGH : LOW);
      break;
    case WIFI_OFF:
      digitalWrite(WIFI_AP_LED_PIN, LOW);
      digitalWrite(WIFI_STA_LED_PIN, LOW);
      break;
  }
  if (WiFi.isConnected()) {
    digitalWrite(WIFI_STA_LED_PIN, ledCounter > 200 && ledCounter < 300 ? HIGH : LOW);
  }

  mqttLoop();
  webLoop();
}

void checkButtons() {
  static uint8_t buttonDown = 0;
  static uint32_t sampleSum = 0;
  static uint16_t sampleCount = 0;
  static uint32_t timerStart = 0;
  if (buttonDown == 0) {
    if ((millis() - timerStart) > TOUCH_RESET_TIME) {
      for (int i=0; i<sizeof(touchPins)/sizeof(touchPins[0]); i++) {
        uint16_t v = touchRead(touchPins[i]);
        if (v < TOUCH_THRESHOLD) {
          buttonDown = touchPins[i];
          sampleSum = v;
          sampleCount = 1;
          timerStart = millis();
          break;
        }
      }
    }
  } else {
    sampleSum += touchRead(buttonDown);
    sampleCount++;
    // check average is still below button down threshold
    if (sampleSum/sampleCount < TOUCH_THRESHOLD) {
      if ((millis() - timerStart) > MIN_TOUCH_TIME) {
        if (handleButton(buttonDown)) {
          timerStart = millis();
        }
        buttonDown = 0;
      }
    } else {
      // button not down long enough
      buttonDown = 0;
    }
  }
}

bool handleButton(uint8_t buttonId) {
  LOGD("handleButton(%d)", buttonId);
  switch (buttonId) {
    case BLE_ADVERT_TOUCH_PIN:
      if (telemetry.config.ble.mode != MODE_DISABLED) {
        if (!bleIsAdvertising) {
          BLEDevice::getAdvertising()->start();
          bleIsAdvertising = true;
          LOGD("Started BLE advertising");
        } else {
          BLEDevice::getAdvertising()->stop();
          bleIsAdvertising = false;
          LOGD("Stopped BLE advertising");
        }
        return true;
      }
      break;
    case WIFI_AP_TOUCH_PIN:
      if (WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
        return startWiFiAP();
      } else {
        return stopWiFi();
      }
      break;
    case WIFI_STA_TOUCH_PIN:
      if (WiFi.getMode() != WIFI_STA && WiFi.getMode() != WIFI_AP_STA) {
        return startWiFiSTA();
      } else {
        return stopWiFi();
      }
      break;
  }
  return false;
}

bool startWiFiAP() {
  WiFi.enableAP(true);
  WiFi.softAPsetHostname(telemetry.config.wifi.ap.hostname);
  if (WiFi.softAP(telemetry.config.wifi.ap.ssid, telemetry.config.wifi.ap.password)) {
    WiFi.softAPConfig(localHost, gateway, subnet);
    webBegin(&telemetry);
    LOGD("Started WiFi AP: %d", WiFi.getMode());
    return true;
  } else {
    LOGE("Failed to start WiFi AP");
    return false;
  }
}

bool startWiFiSTA() {
  WiFi.enableSTA(true);
  WiFi.setHostname(telemetry.config.wifi.client.hostname);
  WiFi.setAutoReconnect(true);
  wl_status_t status = WiFi.begin(telemetry.config.wifi.client.remote.ssid, telemetry.config.wifi.client.remote.password);
  if (status != WL_CONNECT_FAILED) {
    mqttBegin(&telemetry);
    webBegin(&telemetry);
    LOGD("Started WiFi station: %d (%d)", WiFi.getMode(), status);
    return true;
  } else {
    LOGE("Failed to start WiFi station");
    return false;
  }
}

bool stopWiFi() {
  webStop();
  mqttStop();
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  LOGD("Stopped WiFi: %d", WiFi.getMode());
  return true;
}

void processPollPacket(uint8_t physicalId) {
  if (physicalId == PHYSICAL_SENSOR_ID) {
    sendInternalSensors(false);
  }
}

void sendInternalSensors(bool includeStart) {
  if (telemetry.config.internalSensors.enableHallEffect) {
    static uint32_t lastSent = 0;
    if ((millis() - lastSent) > 103 && (INCOMING_CAPACITY - incomingWritePos) > 18) {
      const uint32_t hallValue = hallRead();
      incomingWritePos += writeSensorPacket(incoming+incomingWritePos, PHYSICAL_SENSOR_ID, HALL_EFFECT_ID, hallValue, includeStart);
      if (incomingWritePos > INCOMING_CAPACITY) {
        LOGE("Incoming buffer exceeded! (%d)", incomingWritePos);
      }
      lastSent = millis();
    }
  }
}

void processSensorPacket(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData) {
  static uint8_t sendBuffer[SERIALIZATION_BUFFER_SIZE];
  if (telemetry.config.usb.mode == MODE_FILTER || telemetry.config.bt.mode == MODE_FILTER || telemetry.config.ble.mode == MODE_FILTER) {
    int packetSize = writeSensorPacket(sendBuffer, physicalId, sensorId, sensorData, true);
    if (packetSize <= SERIALIZATION_BUFFER_SIZE) {
      write(sendBuffer, packetSize, MODE_FILTER);
    } else {
      LOGE("Send buffer exceeded! (%d)", packetSize);
    }
  }

  Sensor* sensor = telemetry.updateSensor(physicalId, sensorId, sensorData);
  if (sensor != nullptr) {
    bool sent = false;
    const uint32_t processedAge = (sensor->lastUpdated - sensor->lastProcessed);
    if ((sensor->hasChangedSinceProcessed && processedAge > 200) // throttle frequent changes to no more than every 200ms
    || (processedAge > 500)) { // ensure everything is refreshed every 500ms
      if (webEmitSensor(*sensor)) {
        sent = true;
      }
      if (mqttPublishSensor(*sensor)) {
        sent = true;
      }
      sensor->hasChangedSinceProcessed = false;
      sensor->lastProcessed = millis();
    }
    if (sent) {
      webMsgCount++;
    }
  }
}

Sensor* Telemetry::updateSensor(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData) {
  Sensor* sensor = getSensor(physicalId, sensorId);
  if (sensor == nullptr) {
    if (numSensors < MAX_SENSORS) {
      // add new
      int16_t idx = numSensors++;
      sensor = &(sensors[idx]);
      sensor->_index = idx;
      sensor->physicalId = physicalId;
      sensor->sensorId = sensorId;
      sensor->info = getSensorInfo(sensorId, 0);
    } else {
      return nullptr;
    }
  }
  sensor->setValue(sensorData);
  return sensor;
}

void Sensor::setValue(uint32_t sensorData) {
  if (info) {
    if (info->unit == UNIT_GPS) {
      setGpsValue(sensorData);
    } else {
      setNumericValue(sensorData);
    }
  } else {
      setNumericValue(sensorData);
  }
  lastUpdated = millis();
}

void Sensor::setGpsValue(uint32_t sensorData) {
    int32_t v = (sensorData & 0x3FFFFFFF); // abs value
    if (sensorData & 0x40000000) {
      // restore sign
      v = -v;
    }
    if (sensorData & 0x80000000) { // is longitude
      if (value.gps.longitude != v) {
        lastChangedValue.gps.longitude = v;
        hasChangedSinceProcessed = true;
      }
      value.gps.longitude = v;
    } else {
      if (value.gps.latitude != v) {
        lastChangedValue.gps.latitude = v;
        hasChangedSinceProcessed = true;
      }
      value.gps.latitude = v;
    }
}

void Sensor::setNumericValue(uint32_t sensorData) {
  int32_t v = sensorData;
  if (info == nullptr || info->precision <= 1) {
    if (value.numeric != v) {
      lastChangedValue.numeric = v;
      hasChangedSinceProcessed = true;
    }
  } else if(info->precision > 1) {
    uint32_t diff = abs(v - lastChangedValue.numeric);
    for (int i=0; i<info->precision; i++) {
      diff /= 10;
    }
    if (diff > 0) {
      lastChangedValue.numeric = v;
      hasChangedSinceProcessed = true;
    }
  }
  value.numeric = v;
}

Sensor* Telemetry::getSensor(uint8_t physicalId, uint16_t sensorId) {
  for (int i=0; i<numSensors; i++) {
    if (sensors[i].sensorId == sensorId && sensors[i].physicalId == physicalId) {
      return &(sensors[i]);
    }
  }
  return nullptr;
}

void loadPreferenceString(Preferences& prefs, const char* key, char* value, int maxSize, const char* defaultValue = "") {
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
  config.wifi.mode = (WiFiMode_t) preferences.getShort("wifiMode", WIFI_OFF);
  loadPreferenceString(preferences, "mapTiles", config.map.tiles, URL_SIZE);
  loadPreferenceString(preferences, "mapApiKey", config.map.apiKey, API_KEY_SIZE);
  loadPreferenceString(preferences, "mqttBroker", config.mqtt.broker, ENDPOINT_SIZE);
  config.mqtt.port = preferences.getShort("mqttPort", 8883);
  loadPreferenceString(preferences, "mqttTopic", config.mqtt.topic, TOPIC_SIZE);
  config.internalSensors.enableHallEffect = preferences.getBool("internalSensors", true);
  preferences.end();

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
}

void Telemetry::save() {
  Preferences preferences;
  if (!preferences.begin("telem")) {
    LOGE("Could not open preferences!");
    return;
  }
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
  preferences.putBool("internalSensors", config.internalSensors.enableHallEffect);
  preferences.end();
}
