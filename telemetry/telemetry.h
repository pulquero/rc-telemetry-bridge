#ifndef telemetry_h
#define telemetry_h

#include <WiFiGeneric.h>
#include "sensors.h"
#include "bytes.h"

#define MAX_SENSORS 50
#define INCOMING_CAPACITY 64
#define strncpy_s(dst, src, n) strncpy(dst, src, n);dst[n-1] = '\0';

enum SerialSource {
  SOURCE_NONE, SOURCE_UART, SOURCE_BLE, SOURCE_SOCKET, SOURCE_ESPNOW
};

enum TelemetryProtocol {
  PROTOCOL_NATIVE, PROTOCOL_SMART_PORT, PROTOCOL_CRSF, PROTOCOL_GHST
};

enum SerialMode {
  MODE_DISABLED, MODE_PASS_THRU, MODE_FILTER
};

// sizes including NULL
#define SSID_SIZE 32
#define PASSWORD_SIZE 64
#define NAME_SIZE 16
#define MAC_SIZE 18
#define BD_ADDR_SIZE 18
#define URL_SIZE 128
#define API_KEY_SIZE 128
#define ENDPOINT_SIZE 64
#define TOPIC_SIZE 32

typedef struct {
  struct {
    SerialSource source = SOURCE_UART;
    char btAddress[BD_ADDR_SIZE] = {'\0'};
    TelemetryProtocol protocol = PROTOCOL_SMART_PORT;
  } input;
  struct {
    SerialMode mode;
  } usb;
  struct {
    char name[NAME_SIZE] = {'\0'};
    SerialMode mode;
  } bt;
  struct {
    char name[NAME_SIZE] = {'\0'};
    SerialMode mode;
  } ble;
  struct {
    struct {
      char hostname[NAME_SIZE] = {'\0'};
      uint16_t port;
    } client;
    struct {
      uint16_t port;
      SerialMode mode;
    } server;
  } socket;
  struct {
    char mac[MAC_SIZE] = {'\0'};
    SerialMode mode;
  } espnow;
  struct {
    struct {
      char hostname[NAME_SIZE] = {'\0'};
      char ssid[SSID_SIZE] = {'\0'};
// password must be at least 8 chars
      char password[PASSWORD_SIZE] = {'\0'};
    } ap;
    struct {
      char hostname[NAME_SIZE] = {'\0'};
      struct {
        char ssid[SSID_SIZE] = {'\0'};
        char password[PASSWORD_SIZE] = {'\0'};
      } remote;
    } client;
    WiFiMode_t mode;
  } wifi;
  struct {
    char tiles[URL_SIZE] = {'\0'};
    char apiKey[API_KEY_SIZE] = {'\0'};
  } map;
  struct {
    char broker[ENDPOINT_SIZE] = {'\0'};
    uint16_t port;
    char topic[TOPIC_SIZE] = {'\0'};
  } mqtt;
  struct {
    bool enableHallEffect;
  } internalSensors;
} config_t;

class Telemetry final {
  public:
  config_t config;
  Sensor sensors[MAX_SENSORS];
  int numSensors = 0;
  Buffer<uint8_t,INCOMING_CAPACITY> incoming;
  SerialSource incomingSource;

  void copyToIncoming(const uint8_t* data, size_t len, SerialSource source);
  Sensor* updateSensor(uint8_t physicalId, uint16_t sensorId, uint8_t subId, uint32_t sensorData, SensorDataType sensorDataType);
  Sensor* getSensor(uint8_t physicalId, uint16_t sensorId, uint8_t subId);
  void load();
  void save();
};

#endif
