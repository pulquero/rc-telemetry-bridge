#include "platform.h"
#include "crsf-sensors.h"
#include "crsf-api.h"
#include "json.h"

static int crsfWriteJsonFlightMode(char* out, uint32_t mode);

static const SensorInfo crsfSensorInfos[] = {
  {LINK_ID,        LINK_ID,       0, 10, ZSTR_RX_RSSI1,      UNIT_DB,                0},
  {LINK_ID,        LINK_ID,       1, 10, ZSTR_RX_RSSI2,      UNIT_DB,                0},
  {LINK_ID,        LINK_ID,       2, 10, ZSTR_RX_QUALITY,    UNIT_PERCENT,           0},
  {LINK_ID,        LINK_ID,       3, 10, ZSTR_RX_SNR,        UNIT_DB,                0},
  {LINK_ID,        LINK_ID,       4, 10, ZSTR_ANTENNA,       UNIT_RAW,               0},
  {LINK_ID,        LINK_ID,       5, 10, ZSTR_RF_MODE,       UNIT_RAW,               0},
  {LINK_ID,        LINK_ID,       6, 10, ZSTR_TX_POWER,      UNIT_MILLIWATTS,        0},
  {LINK_ID,        LINK_ID,       7, 10, ZSTR_TX_RSSI,       UNIT_DB,                0},
  {LINK_ID,        LINK_ID,       8, 10, ZSTR_TX_QUALITY,    UNIT_PERCENT,           0},
  {LINK_ID,        LINK_ID,       9, 10, ZSTR_TX_SNR,        UNIT_DB,                0},
  {BATTERY_ID,     BATTERY_ID,     0, 4, ZSTR_BATT,          UNIT_VOLTS,             1},
  {BATTERY_ID,     BATTERY_ID,     1, 4, ZSTR_CURR,          UNIT_AMPS,              1},
  {BATTERY_ID,     BATTERY_ID,     2, 4, ZSTR_CAPACITY,      UNIT_MAH,               0},
  {BATTERY_ID,     BATTERY_ID,     3, 4, ZSTR_BATT_PERCENT,  UNIT_PERCENT,           0},
  {GPS_ID,         GPS_ID,         0, 5, ZSTR_GPS,           UNIT_GPS,               0},
  {GPS_ID,         GPS_ID,         1, 5, ZSTR_GSPD,          UNIT_KMH,               1},
  {GPS_ID,         GPS_ID,         2, 5, ZSTR_HDG,           UNIT_DEGREE,            3},
  {GPS_ID,         GPS_ID,         3, 5, ZSTR_ALT,           UNIT_METERS,            0},
  {GPS_ID,         GPS_ID,         4, 5, ZSTR_SATELLITES,    UNIT_RAW,               0},
  {ATTITUDE_ID,    ATTITUDE_ID,    0, 3, ZSTR_PITCH,         UNIT_RADIANS,           3},
  {ATTITUDE_ID,    ATTITUDE_ID,    1, 3, ZSTR_ROLL,          UNIT_RADIANS,           3},
  {ATTITUDE_ID,    ATTITUDE_ID,    2, 3, ZSTR_YAW,           UNIT_RADIANS,           3},
  {FLIGHT_MODE_ID, FLIGHT_MODE_ID, 0, 1, ZSTR_FLIGHT_MODE,   UNIT_TEXT,              0},
  {CF_VARIO_ID,    CF_VARIO_ID,    0, 1, ZSTR_VSPD,          UNIT_METERS_PER_SECOND, 2},
  {RADIO_ID,       RADIO_ID,       0, 1, "Radio ID",         UNIT_RAW,               0},
  {0,              0,              0, 0, nullptr,            UNIT_RAW,               0},
};

const SensorInfo* crsfGetSensorInfo(uint16_t id, uint8_t subId) {
  for(const SensorInfo *sensorInfo = crsfSensorInfos; sensorInfo->firstId; sensorInfo++) {
    if(sensorInfo->firstId <= id && id <= sensorInfo->lastId && subId == sensorInfo->subId) {
      return sensorInfo;
    }
  }
  return nullptr;
}

int crsfWriteJsonSensorValue(char* out, const Sensor& sensor) {
  if (sensor.sensorId == FLIGHT_MODE_ID) {
    return crsfWriteJsonFlightMode(out, sensor.value.numeric);
  } else if (sensor.info->unit == UNIT_GPS) {
    return jsonWriteGps(out, sensor.value.gps.longitude, sensor.value.gps.latitude);
  } else {
    return jsonWriteNumber(out, sensor.value.numeric, sensor.info->precision);
  }
}

int crsfWriteJsonFlightMode(char* out, uint32_t mode) {
  if (mode < sizeof(flightModeValues)/sizeof(flightModeValues[0])) {
    return sprintf(out, "\"%s\"", flightModeValues[mode]);
  } else {
    return -1;
  }
}
