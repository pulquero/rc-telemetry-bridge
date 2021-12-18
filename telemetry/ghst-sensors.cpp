#include "platform.h"
#include "ghst-sensors.h"
#include "ghst-api.h"
#include "json.h"

static int ghstWriteJsonRfMode(char* out, uint32_t mode);
static int ghstWriteJsonVtxBand(char* out, uint32_t mode);

const SensorInfo ghstSensorInfos[] = {
  {GHOST_ID_RX_RSSI,         GHOST_ID_RX_RSSI,         0, 1, ZSTR_RSSI,             UNIT_DB,                0},
  {GHOST_ID_RX_LQ,           GHOST_ID_RX_LQ,           0, 1, ZSTR_RX_QUALITY,       UNIT_PERCENT,           0},
  {GHOST_ID_RX_SNR,          GHOST_ID_RX_SNR,          0, 1, ZSTR_RX_SNR,           UNIT_DB,                0},

  {GHOST_ID_FRAME_RATE,      GHOST_ID_RX_SNR,          0, 1, ZSTR_FRAME_RATE,       UNIT_RAW,               0},
  {GHOST_ID_TX_POWER,        GHOST_ID_TX_POWER,        0, 1, ZSTR_TX_POWER,         UNIT_MILLIWATTS,        0},
  {GHOST_ID_RF_MODE,         GHOST_ID_RF_MODE,         0, 1, ZSTR_RF_MODE,          UNIT_TEXT,              0},
  {GHOST_ID_TOTAL_LATENCY,   GHOST_ID_TOTAL_LATENCY,   0, 1, ZSTR_TOTAL_LATENCY,    UNIT_RAW,               0},

  {GHOST_ID_VTX_FREQ,        GHOST_ID_VTX_FREQ,        0, 1, ZSTR_VTX_FREQ,         UNIT_RAW,               0},
  {GHOST_ID_VTX_POWER,       GHOST_ID_VTX_POWER,       0, 1, ZSTR_VTX_PWR,          UNIT_RAW,               0},
  {GHOST_ID_VTX_CHAN,        GHOST_ID_VTX_CHAN,        0, 1, ZSTR_VTX_CHAN,         UNIT_RAW,               0},
  {GHOST_ID_VTX_BAND,        GHOST_ID_VTX_BAND,        0, 1, ZSTR_VTX_BAND,         UNIT_TEXT,              0},

  {GHOST_ID_PACK_VOLTS,      GHOST_ID_PACK_VOLTS,      0, 1, ZSTR_BATT,             UNIT_VOLTS,             2},
  {GHOST_ID_PACK_AMPS,       GHOST_ID_PACK_AMPS,       0, 1, ZSTR_CURR,             UNIT_AMPS,              2},
  {GHOST_ID_PACK_MAH,        GHOST_ID_PACK_MAH,        0, 1, ZSTR_CAPACITY,         UNIT_MAH,               0},

  {GHOST_ID_GPS_LAT,         GHOST_ID_GPS_LAT,         0, 1, ZSTR_GPS,              UNIT_GPS_LATITUDE,      0},
  {GHOST_ID_GPS_LONG,        GHOST_ID_GPS_LONG,        0, 1, ZSTR_GPS,              UNIT_GPS_LONGITUDE,     0},
  {GHOST_ID_GPS_GSPD,        GHOST_ID_GPS_GSPD,        0, 1, ZSTR_GSPD,             UNIT_KMH,               1},
  {GHOST_ID_GPS_HDG,         GHOST_ID_GPS_HDG,         0, 1, ZSTR_HDG,              UNIT_DEGREE,            3},
  {GHOST_ID_GPS_ALT,         GHOST_ID_GPS_ALT,         0, 1, ZSTR_ALT,              UNIT_METERS,            0},
  {GHOST_ID_GPS_SATS,        GHOST_ID_GPS_SATS,        0, 1, ZSTR_SATELLITES,       UNIT_RAW,               0},

  {0,                        0,                        0, 0, nullptr,               UNIT_RAW,               0},
};

const SensorInfo* ghstGetSensorInfo(uint16_t id, uint8_t subId) {
  for(const SensorInfo *sensorInfo = ghstSensorInfos; sensorInfo->firstId; sensorInfo++) {
    if(sensorInfo->firstId <= id && id <= sensorInfo->lastId && subId == sensorInfo->subId) {
      return sensorInfo;
    }
  }
  return nullptr;
}

int ghstWriteJsonSensorValue(char* out, const Sensor& sensor) {
  if (sensor.sensorId == GHOST_ID_RF_MODE) {
    return ghstWriteJsonRfMode(out, sensor.value.numeric);
  } else if (sensor.sensorId == GHOST_ID_RF_MODE) {
    return ghstWriteJsonVtxBand(out, sensor.value.numeric);
  } else {
    return jsonWriteNumber(out, sensor.value.numeric, sensor.info->precision);
  }
}

int ghstWriteJsonRfMode(char* out, uint32_t mode) {
  if (mode < sizeof(rfModeValues)/sizeof(rfModeValues[0])) {
    return sprintf(out, "\"%s\"", rfModeValues[mode]);
  } else {
    return -1;
  }
}

int ghstWriteJsonVtxBand(char* out, uint32_t mode) {
  if (mode < sizeof(vtxBandNames)/sizeof(vtxBandNames[0])) {
    return sprintf(out, "\"%s\"", vtxBandNames[mode]);
  } else {
    return -1;
  }
}
