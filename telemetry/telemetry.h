#ifndef telemetry_h
#define telemetry_h

#include <WiFiGeneric.h>
#include "sensors.h"

#define MAX_SENSORS 50
#define strncpy_s(dst, src, n) strncpy(dst, src, n);dst[n-1] = '\0';

union Value {
  int32_t numeric = 0;
  struct Gps {
    int32_t latitude;
    int32_t longitude;
  } gps;
};

class Sensor final {
  private:
  void setGpsValue(uint32_t sensorData);
  void setNumericValue(uint32_t sensorData);
  public:
  int16_t _index = -1;
  uint8_t physicalId = 0;
  uint16_t sensorId = 0;
  union Value value;
  union Value lastChangedValue;
  const SensorInfo *info = nullptr;
  uint32_t lastUpdated = 0;
  bool hasChangedSinceProcessed = false;
  uint32_t lastProcessed = 0;

  void setValue(uint32_t sensorData);
};

enum SerialMode {
  MODE_DISABLED, MODE_PASS_THRU, MODE_FILTER
};

// sizes including NULL
#define SSID_SIZE 32
#define PASSWORD_SIZE 64
#define NAME_SIZE 16
#define URL_SIZE 128
#define API_KEY_SIZE 128
#define ENDPOINT_SIZE 64
#define TOPIC_SIZE 32

typedef struct {
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

  Sensor* updateSensor(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData);
  Sensor* getSensor(uint8_t physicalId, uint16_t sensorId);
  void load();
  void save();
};

#endif
