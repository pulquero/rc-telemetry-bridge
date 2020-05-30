#include <Arduino.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <WiFi.h>
#include <Preferences.h>
#include "protocol.h"
#include "telemetry.h"
#include "web.h"

#include "debug.h"

#define SERVICE_UUID        "0000FFF0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "CHOFA4E1-8155-476F-A173-35389666BE2D"
#define BLE_BUFFER_SIZE 20
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
static BLEServer* bleServer;
static BLECharacteristic* bleChar;
static bool bleIsAdvertising = false;

static uint32_t serialPacketCount = 0;
static uint32_t webMsgCount = 0;

static const uint8_t touchPins[] = {WIFI_STA_TOUCH_PIN, BLE_ADVERT_TOUCH_PIN, WIFI_AP_TOUCH_PIN};
static Preferences preferences;
static Telemetry telemetry;
#define INCOMING_CAPACITY 20
static uint8_t incoming[INCOMING_CAPACITY];
static uint8_t incomingReadPos = 0;
static uint8_t incomingWritePos = 0;

static void checkButtons();
static bool handleButton(uint8_t buttonId);
static void sendInternalSensors(bool includeStart);
static uint16_t getSensorUpdateRate(uint16_t sensorId);
static void loadPreferenceString(const char* key, char* value, int maxSize, const char* defaultValue);

static bool btCongested = false;

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

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  telemetry.load();

// Bluetooth startup sequence is critical!
  if (telemetry.config.ble.mode != MODE_DISABLED) {
    BLEDevice::init(telemetry.config.ble.name);  
    BLEDevice::setPower(ESP_PWR_LVL_P7);
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
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    bleService->start();
    BLEAdvertising* bleAdvertising = BLEDevice::getAdvertising();
    bleAdvertising->addServiceUUID(SERVICE_UUID);
    bleAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
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
    for (int i=0; i<len && bleWritePos<BLE_BUFFER_SIZE; i++) {
      bleBuffer[bleWritePos++] = data[i];
    }
    if (bleWritePos == BLE_BUFFER_SIZE) {
      bleChar->setValue(bleBuffer, BLE_BUFFER_SIZE);
      bleChar->notify();
      bleWritePos = 0;
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
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAPsetHostname(telemetry.config.wifi.ap.hostname);
        WiFi.softAP(telemetry.config.wifi.ap.ssid, telemetry.config.wifi.ap.password);
        WiFi.softAPConfig(localHost, gateway, subnet);
        webBegin(&telemetry);
        LOGD("Started WiFi AP");
        return true;
      } else {
        webStop();
        WiFi.mode(WIFI_OFF);
        LOGD("Stopped WiFi AP");
        return true;
      }
      break;
    case WIFI_STA_TOUCH_PIN:
      if (!WiFi.isConnected()) {
        if (WiFi.getMode() != WIFI_STA && WiFi.getMode() != WIFI_AP_STA) {
          WiFi.mode(WIFI_STA);
        }
        WiFi.setHostname(telemetry.config.wifi.client.hostname);
        WiFi.setAutoReconnect(true);
        WiFi.begin(telemetry.config.wifi.client.remote.ssid, telemetry.config.wifi.client.remote.password);
        webBegin(&telemetry);
        LOGD("Started WiFi station");
        return true;
      }
      break;
  }
  return false;
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
      incomingWritePos += writeSensorPacket(incoming+incomingWritePos, PHYSICAL_SENSOR_ID, HALL_EFFECT_ID, hallRead(), includeStart);
      if (incomingWritePos > INCOMING_CAPACITY) {
        LOGE("Incoming buffer exceeded!");
      }
      lastSent = millis();
    }
  }
}

void processSensorPacket(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData) {
  static uint8_t sendBuffer[20];
  if (telemetry.config.usb.mode == MODE_FILTER || telemetry.config.bt.mode == MODE_FILTER || telemetry.config.ble.mode == MODE_FILTER) {
    int packetSize = writeSensorPacket(sendBuffer, physicalId, sensorId, sensorData, true);
    if (packetSize > 20) {
      LOGE("Send buffer exceeded!");
    }
    write(sendBuffer, packetSize, MODE_FILTER);
  }

  Sensor* sensor = telemetry.updateSensor(physicalId, sensorId, sensorData);
  if (sensor != nullptr) {
    if ((sensor->lastUpdated - sensor->lastSent) > getSensorUpdateRate(sensorId)) {
      if (webEmitSensor(*sensor)) {
        webMsgCount++;
        sensor->lastSent = millis();
      }
    }
  }
}

uint16_t getSensorUpdateRate(uint16_t sensorId) {
  if (sensorId >= VFAS_FIRST_ID && sensorId <= VFAS_LAST_ID) {
    return 1000;
  } else if (sensorId >= T1_FIRST_ID && sensorId <= T1_LAST_ID) {
    return 1000;
  } else if (sensorId >= T2_FIRST_ID && sensorId <= T2_LAST_ID) {
    return 1000;
  } else if (sensorId >= ESC_TEMPERATURE_FIRST_ID && sensorId <= ESC_TEMPERATURE_LAST_ID) {
    return 1000;
  } else if (sensorId >= ESC_TEMPERATURE_FIRST_ID && sensorId <= ESC_TEMPERATURE_LAST_ID) {
    return 1000;
  } else {
    return 500;
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
      int32_t v = (sensorData & 0x3FFFFFFF); // abs value
      if (sensorData & 0x40000000) {
        // restore sign
        v = -v;
      }
      if (sensorData & 0x80000000) {
        value.gps.longitude = v;
      } else {
        value.gps.latitude = v;
      }
    } else {
      value.numeric = sensorData;
    }
  } else {
    value.numeric = sensorData;
  }
  lastUpdated = millis();
}

Sensor* Telemetry::getSensor(uint8_t physicalId, uint16_t sensorId) {
  for (int i=0; i<numSensors; i++) {
    if (sensors[i].sensorId == sensorId && sensors[i].physicalId == physicalId) {
      return &(sensors[i]);
    }
  }
  return nullptr;
}

void loadPreferenceString(const char* key, char* value, int maxSize, const char* defaultValue = "") {
  if (!preferences.getString(key, value, maxSize)) {
    strncpy_s(value, defaultValue, maxSize);
  }
}

void Telemetry::load() {
  if (!preferences.begin("telem")) {
    LOGE("Could not open preferences!");
    return;
  }
  // emergency reset
  //preferences.clear();
  config.usb.mode = (SerialMode) preferences.getShort("usbMode", MODE_PASS_THRU);
  loadPreferenceString("btName", config.bt.name, NAME_SIZE, "Telemetry BT");
  config.bt.mode = (SerialMode) preferences.getShort("btMode", MODE_DISABLED);
  loadPreferenceString("bleName", config.ble.name, NAME_SIZE, "Telemetry BLE");
  config.ble.mode = (SerialMode) preferences.getShort("bleMode", MODE_FILTER);
  loadPreferenceString("wifiApHostname", config.wifi.ap.hostname, NAME_SIZE, "telemetry");
  loadPreferenceString("wifiApSsid", config.wifi.ap.ssid, SSID_SIZE, "Telemetry WiFi");
  loadPreferenceString("wifiApPassword", config.wifi.ap.password, PASSWORD_SIZE);
  loadPreferenceString("wifiHostname", config.wifi.client.hostname, NAME_SIZE, "telemetry");
  loadPreferenceString("wifiStaSsid", config.wifi.client.remote.ssid, SSID_SIZE);
  loadPreferenceString("wifiStaPassword", config.wifi.client.remote.password, PASSWORD_SIZE);
  loadPreferenceString("mapTiles", config.map.tiles, URL_SIZE);
  loadPreferenceString("mapApiKey", config.map.apiKey, API_KEY_SIZE);
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
}

void Telemetry::save() {
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
  preferences.putString("mapTiles", config.map.tiles);
  preferences.putString("mapApiKey", config.map.apiKey);
  preferences.putBool("internalSensors", config.internalSensors.enableHallEffect);
  preferences.end();
}
