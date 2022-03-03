// No OTA (2MB APP/2MB SPIFFS)

#define USE_BLE_SERVER
#define USE_BLE_CLIENT
//#define USE_BT_SPP
#define USE_NIMBLE

#ifdef USE_NIMBLE
#undef USE_BT_SPP
#endif

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPIFFS.h>
#ifdef USE_BT_SPP
#include <BluetoothSerial.h>
#endif
#if defined(USE_BLE_SERVER) || defined(USE_BLE_CLIENT)
#ifdef USE_NIMBLE
#include <NimBLEDevice.h>
#else
#include <BLEDevice.h>
#include <BLE2902.h>
#endif
#endif
#include <WiFi.h>
#include "protocol.h"
#include "telemetry.h"
#include "smartport.h"
#include "smartport-sensors.h"
#include "crsf.h"
#include "ghst.h"
#include "web.h"
#include "mqtt-transport.h"
#include "socket-transport.h"

#include "debug.h"

#define SERIALIZED_PACKET_SIZE (1+SPORT_DATA_PACKET_LEN+2+1) // allow for two byte stuffings
#define SERIALIZATION_BUFFER_SIZE 64 // large enough to hold biggest packet (of any protocol)
#define SERVICE_UUID        "0000FFF0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFF6-0000-1000-8000-00805F9B34FB"
#define MAX_MTU_SIZE 65
#define BLE_BUFFER_SIZE MAX_MTU_SIZE-MTU_OVERHEAD
#define MTU_OVERHEAD 3
#define BLE_RECONNECT_DELAY 2000 // must be greater than connect timeout
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

#ifdef USE_BLE_CLIENT
static BLEClient* bleClient;
#endif

#ifdef USE_BT_SPP
static BluetoothSerial* btSerial;
#endif
static bool btCongested = false;
#ifdef USE_BLE_SERVER
static BLEServer* bleServer;
static BLECharacteristic* bleChar;
static bool bleIsAdvertising = false;
static int blePacketDataSize = 23 - MTU_OVERHEAD;
#endif

static uint32_t serialPacketCount = 0;
static uint32_t webMsgCount = 0;

static const uint8_t touchPins[] = {WIFI_STA_TOUCH_PIN, BLE_ADVERT_TOUCH_PIN, WIFI_AP_TOUCH_PIN};
static Telemetry telemetry;

static uint8_t sendBuffer[SERIALIZATION_BUFFER_SIZE];

static void write(uint8_t* data, int len, SerialMode mode);
static void checkButtons(uint32_t ms);
static bool handleButton(uint8_t buttonId);
static bool startWiFiAP();
static bool startWiFiSTA();
static bool stopWiFi();
static void sendInternalSensors(bool includeStart);


#ifdef USE_BT_SPP
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
    default:
      ;
  }
}
#endif

#ifdef USE_BLE_SERVER
#ifdef USE_NIMBLE
int bleTelemetryCallback(struct ble_gap_event* event, void* param) {
  switch(event->type) {
    case BLE_GAP_EVENT_MTU:
      LOGD("BLE MTU set to %d", event->mtu.value);
      // ensure we don't exceed the buffer size
      blePacketDataSize = min(event->mtu.value - MTU_OVERHEAD, BLE_BUFFER_SIZE);
      break;
    default:
      ;
  }
  return 0;
}
#else
void bleTelemetryCallback(esp_gatts_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gatts_cb_param_t* param) {
  switch(event) {
    case ESP_GATTS_MTU_EVT:
      LOGD("BLE MTU set to %d", param->mtu.mtu);
      // ensure we don't exceed the buffer size
      blePacketDataSize = min(param->mtu.mtu - MTU_OVERHEAD, BLE_BUFFER_SIZE);
      break;
    default:
      ;
  }
}
#endif
#endif

#ifdef USE_BLE_CLIENT
void bleSourceCallback(BLERemoteCharacteristic* remoteChar, uint8_t* data, size_t len, bool isNotify) {
  telemetry.copyToIncoming(data, len, SOURCE_BLE);
}

bool bleSourceConnect() {
  bool subscribed = false;
  LOGD("Trying to connect to BLE device %s", telemetry.config.input.btAddress);
#ifdef USE_NIMBLE
  bool rc = bleClient->connect(BLEAddress(telemetry.config.input.btAddress));
#else
  bool rc = bleClient->connect(BLEAddress(telemetry.config.input.btAddress));
#endif
  if (rc) {
    LOGD("BLE client connected to %s", telemetry.config.input.btAddress);
    bleClient->getServices(true);
    BLERemoteService* remoteService = bleClient->getService(SERVICE_UUID);
    if (remoteService) {
      remoteService->getCharacteristics(true);
      BLERemoteCharacteristic* remoteChar = remoteService->getCharacteristic(CHARACTERISTIC_UUID);
      if (remoteChar && remoteChar->canNotify()) {
#ifdef USE_NIMBLE
          if (remoteChar->subscribe(true, bleSourceCallback)) {
              LOGD("Subscribed to notifications for BLE characteristic %s", CHARACTERISTIC_UUID);
              subscribed = true;
          } else {
              LOGD("Failed to subscribe to notifications for BLE characteristic %s", CHARACTERISTIC_UUID);
              bleClient->disconnect();
          }
#else
          remoteChar->registerForNotify(bleSourceCallback);
          LOGD("Notifications enabled for BLE characteristic %s", CHARACTERISTIC_UUID);
          subscribed = true;
#endif
      } else {
        LOGE("BLE characteristic %s not found or does not support notify", CHARACTERISTIC_UUID);
        bleClient->disconnect();
      }
    } else {
      LOGE("BLE service %s not found", SERVICE_UUID);
      bleClient->disconnect();
    }
  } else {
    LOGE("Failed to connect to BLE device %s", telemetry.config.input.btAddress);
  }
  LOGMEM("post-bleSourceConnect");
  return subscribed;
}
#endif

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  LOGMEM("boot");
  telemetry.load();
  protocolBegin(&telemetry);
  LOGMEM("post-config");

// Bluetooth startup sequence is critical!
#if defined(USE_BLE_SERVER) || defined(USE_BLE_CLIENT)
  if (telemetry.config.ble.mode != MODE_DISABLED || telemetry.config.input.source == SOURCE_BLE) {
    BLEDevice::init(telemetry.config.ble.name);
    BLEDevice::setPower(ESP_PWR_LVL_P7);
    LOGD("BLE local address: %s", BLEDevice::getAddress().toString().c_str());
    LOGMEM("post-BLEDevice");
  }
#endif

#ifdef USE_BT_SPP
  if (telemetry.config.bt.mode != MODE_DISABLED) {
    btSerial = new BluetoothSerial();
    btSerial->register_callback(btTelemetryCallback);
    if(!btSerial->begin(telemetry.config.bt.name)) {
      LOGE("Failed to initialise BT");
    }
  }
#endif

#ifdef USE_BLE_SERVER
  if (telemetry.config.ble.mode != MODE_DISABLED) {
    // server MTU
    BLEDevice::setMTU(MAX_MTU_SIZE); // max 517
#ifdef USE_NIMBLE
    BLEDevice::setCustomGapHandler(bleTelemetryCallback);
#else
    BLEDevice::setCustomGattsHandler(bleTelemetryCallback);
#endif
    bleServer = BLEDevice::createServer();
    if (!bleServer) {
      LOGE("Failed to create BLE server");
    }
    BLEService* bleService = bleServer->createService(SERVICE_UUID);
    bleChar = bleService->createCharacteristic(CHARACTERISTIC_UUID,
#ifdef USE_NIMBLE
      NIMBLE_PROPERTY::NOTIFY
#else
      BLECharacteristic::PROPERTY_NOTIFY
#endif
    );
#ifndef USE_NIMBLE
    BLE2902* ble2902Desc = new BLE2902();
    ble2902Desc->setNotifications(true);
    bleChar->addDescriptor(ble2902Desc);
#endif
    bleService->start();
    BLEAdvertising* bleAdvertising = BLEDevice::getAdvertising();
    bleAdvertising->addServiceUUID(SERVICE_UUID);
    bleAdvertising->setScanResponse(true);
  }
#endif

#ifdef USE_BLE_CLIENT
  if (telemetry.config.input.source == SOURCE_BLE && strlen(telemetry.config.input.btAddress) > 0) {
    bleClient = BLEDevice::createClient();
    if (bleClient) {
#ifdef USE_NIMBLE
      bleClient->setConnectTimeout(1);
#endif
    } else {
      LOGE("Failed to create BLE client");
    }
  }
#endif

  // autostart

  SPIFFS.begin();
#ifdef USE_BLE_SERVER
  if (telemetry.config.ble.mode != MODE_DISABLED) {
    BLEDevice::startAdvertising();
  }
#endif
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

  LOGMEM("post-setup");
}

void bleWrite(uint8_t* data, int len, SerialMode mode) {
  static Buffer<uint8_t,BLE_BUFFER_SIZE> bleBuffer;
  if (mode == MODE_FILTER) {
    // send complete telemetry packets
    // if it doesn't fit and there is already data in the buffer then send it
    if (!bleBuffer.isEmpty() && !bleBuffer.fitsWithin(blePacketDataSize - len)) {
      bleChar->setValue(bleBuffer.data(), bleBuffer.size());
      bleChar->notify();
      bleBuffer.reset();
    }
    // can the buffer + len fit within blePacketDataSize
    if (bleBuffer.fitsWithin(blePacketDataSize - len)) {
      bleBuffer.copyFrom(data, len);
    } else {
      LOGE("Discarding BLE data - exceeds MTU");
    }
    // send data now if there is no more space to fit further packets
    if (!bleBuffer.fitsWithin(blePacketDataSize - SERIALIZED_PACKET_SIZE)) {
      bleChar->setValue(bleBuffer.data(), bleBuffer.size());
      bleChar->notify();
      bleBuffer.reset();
    }
  } else {
    for (int i=0; i<len; i++) {
      bleBuffer.add(data[i]);
      if (bleBuffer.size() == blePacketDataSize) {
        bleChar->setValue(bleBuffer.data(), bleBuffer.size());
        bleChar->notify();
        bleBuffer.reset();
      }
    }
  }
}

void write(uint8_t* data, int len, SerialMode mode) {
#ifndef DEBUG
  if (telemetry.config.usb.mode == mode) {
    if (mode == MODE_FILTER && telemetry.config.input.protocol == PROTOCOL_SMART_PORT) {
      // skip initial START_STOP - previous packet has trailing START_STOP
      data++;
      len--;
    }
    Serial.write(data, len);
  }
#endif
#ifdef USE_BT_SPP
  if (telemetry.config.bt.mode == mode && !btCongested) {
    if (mode == MODE_FILTER && telemetry.config.input.protocol == PROTOCOL_SMART_PORT) {
      // skip initial START_STOP
      data++;
      len--;
    }
    btSerial->write(data, len);
  }
#endif
#ifdef USE_BLE_SERVER
  if (telemetry.config.ble.mode == mode) {
    bleWrite(data, len, mode);
  }
#endif
  if (telemetry.config.socket.server.mode == mode) {
    socketWrite(data, len);
  }
}

void bleEnsureConnected(uint32_t ms) {
  static uint32_t lastConnectAttempt = 0;
  if (bleClient && !bleClient->isConnected() && (ms - lastConnectAttempt) > BLE_RECONNECT_DELAY) {
    if (bleSourceConnect()) {
      lastConnectAttempt = 0;
    } else {
      lastConnectAttempt = ms;
    }
  }
}

void loop() {
  static uint32_t lastSerialCheckpoint = 0;
  const uint32_t ms = millis();

  if (!telemetry.incoming.isEmpty()) {
    write(telemetry.incoming.data(), telemetry.incoming.size(), MODE_PASS_THRU);
    while (!telemetry.incoming.isEmpty()) {
      if (protocolOnReceive(telemetry.incoming.read()) > 0 && telemetry.incomingSource > 0) {
        serialPacketCount++;
      }
    }
    telemetry.incoming.reset();
  } else if (telemetry.config.input.source == SOURCE_UART) {
    // rx is floating by default so when nothing is connected we may read noise
    if (Serial2.available()) {
      static uint8_t buf[1];
      buf[0] = Serial2.read();
      write(buf, 1, MODE_PASS_THRU);
      if (protocolOnReceive(buf[0]) > 0) {
        serialPacketCount++;
      }
    }
    if ((ms - lastSerialCheckpoint) > 97) {
      if (serialPacketCount == 0) {
        // received nothing or garbage so resort to sending our own sensors
        sendInternalSensors(true);
      }
      serialPacketCount = 0;
      lastSerialCheckpoint = ms;
    }
  } else if (telemetry.config.input.source == SOURCE_BLE) {
    bleEnsureConnected(ms);
  } else if (telemetry.config.input.source == SOURCE_SOCKET) {
    socketEnsureConnected(ms);
  }

  checkButtons(ms);

  uint32_t ledCounter = ms%400;
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
#ifdef USE_BLE_SERVER
  if (bleIsAdvertising) {
    digitalWrite(BLE_ADVERT_LED_PIN, ledCounter > 300 ? HIGH : LOW);
  } else {
    digitalWrite(BLE_ADVERT_LED_PIN, LOW);
  }
#endif
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
    default:
      ;
  }
  if (WiFi.isConnected()) {
    digitalWrite(WIFI_STA_LED_PIN, ledCounter > 200 && ledCounter < 300 ? HIGH : LOW);
  }

  mqttLoop();
  webLoop(ms);
}

void checkButtons(const uint32_t ms) {
  static uint8_t buttonDown = 0;
  static uint32_t sampleSum = 0;
  static uint16_t sampleCount = 0;
  static uint32_t timerStart = 0;
  if (buttonDown == 0) {
    if ((ms - timerStart) > TOUCH_RESET_TIME) {
      for (int i=0; i<sizeof(touchPins)/sizeof(touchPins[0]); i++) {
        uint16_t v = touchRead(touchPins[i]);
        if (v < TOUCH_THRESHOLD) {
          buttonDown = touchPins[i];
          sampleSum = v;
          sampleCount = 1;
          timerStart = ms;
          break;
        }
      }
    }
  } else {
    sampleSum += touchRead(buttonDown);
    sampleCount++;
    // check average is still below button down threshold
    if (sampleSum/sampleCount < TOUCH_THRESHOLD) {
      if ((ms - timerStart) > MIN_TOUCH_TIME) {
        if (handleButton(buttonDown)) {
          timerStart = ms;
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
#ifdef USE_BLE_SERVER
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
#endif
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
    socketBegin(&telemetry);
    LOGD("Started WiFi AP: mode %d", WiFi.getMode());
    return true;
  } else {
    LOGE("Failed to start WiFi AP");
    return false;
  }
}

bool startWiFiSTA() {
  WiFi.setHostname(telemetry.config.wifi.client.hostname);
  WiFi.enableSTA(true);
  WiFi.setAutoReconnect(true);
  wl_status_t status = WiFi.begin(telemetry.config.wifi.client.remote.ssid, telemetry.config.wifi.client.remote.password);
  if (status != WL_CONNECT_FAILED) {
    mqttBegin(&telemetry);
    webBegin(&telemetry);
    socketBegin(&telemetry);
    LOGD("Started WiFi station: mode %d (status %d)", WiFi.getMode(), status);
    return true;
  } else {
    LOGE("Failed to start WiFi station");
    return false;
  }
}

bool stopWiFi() {
  socketStop();
  webStop();
  mqttStop();
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  LOGD("Stopped WiFi: mode %d", WiFi.getMode());
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
    if ((millis() - lastSent) > 103 && telemetry.incoming.remaining() > 18) {
      const uint32_t hallValue = hallRead();
      const int bytesWritten = sportWriteSensorPacket(telemetry.incoming.newData(), PHYSICAL_SENSOR_ID, HALL_EFFECT_ID, hallValue, includeStart);
      telemetry.incoming.supplied(bytesWritten);
      if (telemetry.incoming.remaining() < 0) {
        LOGE("Incoming buffer exceeded! (%d)", telemetry.incoming.remaining());
      }
      telemetry.incomingSource = SOURCE_NONE;
      lastSent = millis();
    }
  }
}

bool isFilterModeActive() {
  return (telemetry.config.usb.mode == MODE_FILTER
        || telemetry.config.bt.mode == MODE_FILTER
        || telemetry.config.ble.mode == MODE_FILTER
        || telemetry.config.socket.server.mode == MODE_FILTER);
}

void outputSPortSensorPacket(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData) {
  if (isFilterModeActive()) {
    const int packetSize = sportWriteSensorPacket(sendBuffer, physicalId, sensorId, sensorData, true);
    if (packetSize <= SERIALIZATION_BUFFER_SIZE) {
      write(sendBuffer, packetSize, MODE_FILTER);
    } else {
      LOGE("Send buffer exceeded (SPort)! (%d)", packetSize);
    }
  }
}

void outputCrsfSensorPacket(uint8_t deviceAddr, uint8_t frameType, uint8_t* frameData, int dataLen) {
  if (isFilterModeActive()) {
    const int packetSize = crsfWriteSensorPacket(sendBuffer, deviceAddr, frameType, frameData, dataLen);
    if (packetSize <= SERIALIZATION_BUFFER_SIZE) {
      write(sendBuffer, packetSize, MODE_FILTER);
    } else {
      LOGE("Send buffer exceeded (CRSF)! (%d)", packetSize);
    }
  }
}

void outputGhstSensorPacket(uint8_t deviceAddr, uint8_t frameType, uint8_t* frameData, int dataLen) {
  if (isFilterModeActive()) {
    const int packetSize = ghstWriteSensorPacket(sendBuffer, deviceAddr, frameType, frameData, dataLen);
    if (packetSize <= SERIALIZATION_BUFFER_SIZE) {
      write(sendBuffer, packetSize, MODE_FILTER);
    } else {
      LOGE("Send buffer exceeded (GHST)! (%d)", packetSize);
    }
  }
}

void processSensorPacket(uint8_t physicalId, uint16_t sensorId, uint8_t subId, uint32_t sensorData, SensorDataType sensorDataType) {
  Sensor* sensor = telemetry.updateSensor(physicalId, sensorId, subId, sensorData, sensorDataType);
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

