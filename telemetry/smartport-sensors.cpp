#ifdef ESP32_PLATFORM
#include <Arduino.h>
#else
#include <stdio.h>
#endif
#include "smartport-sensors.h"
#include "smartport-api.h"
#include "json.h"

static int sportWriteJsonFlightModeArray(char* out, uint32_t modes);
static int sportWriteJsonGpsStateArray(char* out, uint32_t modes);

static const SensorInfo sportSensorInfos[] = {
  { VALID_FRAME_RATE_ID, VALID_FRAME_RATE_ID, 0, 1, ZSTR_VFR, UNIT_PERCENT, 0 },
  { RSSI_ID, RSSI_ID, 0, 1, ZSTR_RSSI, UNIT_DB, 0 },
  { ADC1_ID, ADC1_ID, 0, 1, ZSTR_A1, UNIT_VOLTS, 1 },
  { ADC2_ID, ADC2_ID, 0, 1, ZSTR_A2, UNIT_VOLTS, 1 },
  { A3_FIRST_ID, A3_LAST_ID, 0, 1, ZSTR_A3, UNIT_VOLTS, 2 },
  { A4_FIRST_ID, A4_LAST_ID, 0, 1, ZSTR_A4, UNIT_VOLTS, 2 },
  { BATT_ID, BATT_ID, 0, 1, ZSTR_BATT, UNIT_VOLTS, 1 },
  { R9_PWR_ID, R9_PWR_ID, 0, 1, ZSTR_R9PW, UNIT_MILLIWATTS, 0 },
#if defined(BETAFLIGHT_SENSORS) || defined(INAV_SENSORS)
  { FLIGHT_MODE_ID, FLIGHT_MODE_ID, 0, 1, ZSTR_FLIGHT_MODE, UNIT_RAW, 0 },
#endif
  { T1_FIRST_ID, T1_LAST_ID, 0, 1, ZSTR_TEMP1, UNIT_CELSIUS, 0 },
#if defined(BETAFLIGHT_SENSORS) || defined(INAV_SENSORS)
  { GPS_STATE_ID, GPS_STATE_ID, 0, 1, "GPS state", UNIT_RAW, 0 },
#endif
  { T2_FIRST_ID, T2_LAST_ID, 0, 1, ZSTR_TEMP2, UNIT_CELSIUS, 0 },
  { RPM_FIRST_ID, RPM_LAST_ID, 0, 1, ZSTR_RPM, UNIT_RPMS, 0 },
  { FUEL_FIRST_ID, FUEL_LAST_ID, 0, 1, ZSTR_FUEL, UNIT_MAH, 0 },
  { ALT_FIRST_ID, ALT_LAST_ID, 0, 1, ZSTR_ALT, UNIT_METERS, 2 },
  { VARIO_FIRST_ID, VARIO_LAST_ID, 0, 1, ZSTR_VSPD, UNIT_METERS_PER_SECOND, 2 },
  { ACCX_FIRST_ID, ACCX_LAST_ID, 0, 1, ZSTR_ACCX, UNIT_G, 2 },
  { ACCY_FIRST_ID, ACCY_LAST_ID, 0, 1, ZSTR_ACCY, UNIT_G, 2 },
  { ACCZ_FIRST_ID, ACCZ_LAST_ID, 0, 1, ZSTR_ACCZ, UNIT_G, 2 },
  { CURR_FIRST_ID, CURR_LAST_ID, 0, 1, ZSTR_CURR, UNIT_AMPS, 1 },
  { VFAS_FIRST_ID, VFAS_LAST_ID, 0, 1, ZSTR_VFAS, UNIT_VOLTS, 2 },
  { AIR_SPEED_FIRST_ID, AIR_SPEED_LAST_ID, 0, 1, ZSTR_ASPD, UNIT_KTS, 1 },
  { GPS_SPEED_FIRST_ID, GPS_SPEED_LAST_ID, 0, 1, ZSTR_GSPD, UNIT_KTS, 3 },
  { CELLS_FIRST_ID, CELLS_LAST_ID, 0, 1, ZSTR_CELLS, UNIT_CELLS, 2 },
  { GPS_ALT_FIRST_ID, GPS_ALT_LAST_ID, 0, 1, ZSTR_GPSALT, UNIT_METERS, 2 },
  { GPS_TIME_DATE_FIRST_ID, GPS_TIME_DATE_LAST_ID, 0, 1, ZSTR_GPSDATETIME, UNIT_DATETIME, 0 },
  { GPS_LONG_LATI_FIRST_ID, GPS_LONG_LATI_LAST_ID, 0, 1, ZSTR_GPS, UNIT_GPS, 0 },
  { FUEL_QTY_FIRST_ID, FUEL_QTY_LAST_ID, 0, 1, ZSTR_FUEL, UNIT_MILLILITERS, 2 },
  { GPS_COURS_FIRST_ID, GPS_COURS_LAST_ID, 0, 1, ZSTR_HDG, UNIT_DEGREE, 2 },
  { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 0, 2, ZSTR_BATT1_VOLTAGE, UNIT_VOLTS, 3 },
  { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 1, 2, ZSTR_BATT1_CURRENT, UNIT_AMPS, 2 },
  { RBOX_BATT2_FIRST_ID, RBOX_BATT2_LAST_ID, 0, 2, ZSTR_BATT2_VOLTAGE, UNIT_VOLTS, 3 },
  { RBOX_BATT2_FIRST_ID, RBOX_BATT2_LAST_ID, 1, 2, ZSTR_BATT2_CURRENT, UNIT_AMPS, 2 },
  { RBOX_CNSP_FIRST_ID, RBOX_CNSP_LAST_ID, 0, 2, ZSTR_BATT1_CONSUMPTION, UNIT_MAH, 0 },
  { RBOX_CNSP_FIRST_ID, RBOX_CNSP_LAST_ID, 1, 2, ZSTR_BATT2_CONSUMPTION, UNIT_MAH, 0 },
  { RBOX_STATE_FIRST_ID, RBOX_STATE_LAST_ID, 0, 2, ZSTR_CHANS_STATE, UNIT_BITFIELD, 0 },
  { RBOX_STATE_FIRST_ID, RBOX_STATE_LAST_ID, 1, 2, ZSTR_RB_STATE, UNIT_BITFIELD, 0 },
  { SD1_FIRST_ID, SD1_LAST_ID, 0, 1, ZSTR_SD1_CHANNEL, UNIT_RAW, 0 },
  { ESC_POWER_FIRST_ID, ESC_POWER_LAST_ID, 0, 2, ZSTR_ESC_VOLTAGE, UNIT_VOLTS, 2 },
  { ESC_POWER_FIRST_ID, ESC_POWER_LAST_ID, 1, 2, ZSTR_ESC_CURRENT, UNIT_AMPS, 2 },
  { ESC_RPM_CONS_FIRST_ID, ESC_RPM_CONS_LAST_ID, 0, 2, ZSTR_ESC_RPM, UNIT_RPMS, 0 },
  { ESC_RPM_CONS_FIRST_ID, ESC_RPM_CONS_LAST_ID, 1, 2, ZSTR_ESC_CONSUMPTION, UNIT_MAH, 0 },
  { ESC_TEMPERATURE_FIRST_ID, ESC_TEMPERATURE_LAST_ID, 0, 1, ZSTR_ESC_TEMP, UNIT_CELSIUS, 0 },
  { GASSUIT_TEMP1_FIRST_ID, GASSUIT_TEMP1_LAST_ID, 0, 1, ZSTR_GASSUIT_TEMP1, UNIT_CELSIUS, 0 },
  { GASSUIT_TEMP2_FIRST_ID, GASSUIT_TEMP2_LAST_ID, 0, 1, ZSTR_GASSUIT_TEMP2, UNIT_CELSIUS, 0 },
  { GASSUIT_SPEED_FIRST_ID, GASSUIT_SPEED_LAST_ID, 0, 1, ZSTR_GASSUIT_RPM, UNIT_RPMS, 0 },
  { GASSUIT_RES_VOL_FIRST_ID, GASSUIT_RES_VOL_LAST_ID, 0, 1, ZSTR_GASSUIT_RES_VOL, UNIT_MILLILITERS, 0 },
  { GASSUIT_RES_PERC_FIRST_ID, GASSUIT_RES_PERC_LAST_ID, 0, 1, ZSTR_GASSUIT_RES_PERC, UNIT_PERCENT, 0 },
  { GASSUIT_FLOW_FIRST_ID, GASSUIT_FLOW_LAST_ID, 0, 1, ZSTR_GASSUIT_FLOW, UNIT_MILLILITERS_PER_MINUTE, 0 },
  { GASSUIT_MAX_FLOW_FIRST_ID, GASSUIT_MAX_FLOW_LAST_ID, 0, 1, ZSTR_GASSUIT_MAX_FLOW, UNIT_MILLILITERS_PER_MINUTE, 0 },
  { GASSUIT_AVG_FLOW_FIRST_ID, GASSUIT_AVG_FLOW_LAST_ID, 0, 1, ZSTR_GASSUIT_AVG_FLOW, UNIT_MILLILITERS_PER_MINUTE, 0 },
  { SBEC_POWER_FIRST_ID, SBEC_POWER_LAST_ID, 0, 2, ZSTR_SBEC_VOLTAGE, UNIT_VOLTS, 2 },
  { SBEC_POWER_FIRST_ID, SBEC_POWER_LAST_ID, 1, 2, ZSTR_SBEC_CURRENT, UNIT_AMPS, 2 },
  { RB3040_OUTPUT_FIRST_ID, RB3040_OUTPUT_LAST_ID, 0, 1, ZSTR_RB3040_EXTRA_STATE, UNIT_BITFIELD, 0 },
  { RB3040_CH1_2_FIRST_ID, RB3040_CH1_2_LAST_ID, 0, 2, ZSTR_RB3040_CHANNEL1, UNIT_AMPS, 2 },
  { RB3040_CH1_2_FIRST_ID, RB3040_CH1_2_LAST_ID, 1, 2, ZSTR_RB3040_CHANNEL2, UNIT_AMPS, 2 },
  { RB3040_CH3_4_FIRST_ID, RB3040_CH3_4_LAST_ID, 0, 2, ZSTR_RB3040_CHANNEL3, UNIT_AMPS, 2 },
  { RB3040_CH3_4_FIRST_ID, RB3040_CH3_4_LAST_ID, 1, 2, ZSTR_RB3040_CHANNEL4, UNIT_AMPS, 2 },
  { RB3040_CH5_6_FIRST_ID, RB3040_CH5_6_LAST_ID, 0, 2, ZSTR_RB3040_CHANNEL5, UNIT_AMPS, 2 },
  { RB3040_CH5_6_FIRST_ID, RB3040_CH5_6_LAST_ID, 1, 2, ZSTR_RB3040_CHANNEL6, UNIT_AMPS, 2 },
  { RB3040_CH7_8_FIRST_ID, RB3040_CH7_8_LAST_ID, 0, 2, ZSTR_RB3040_CHANNEL7, UNIT_AMPS, 2 },
  { RB3040_CH7_8_FIRST_ID, RB3040_CH7_8_LAST_ID, 1, 2, ZSTR_RB3040_CHANNEL8, UNIT_AMPS, 2 },
  { SERVO_FIRST_ID, SERVO_LAST_ID, 0, 4, ZSTR_SERVO_CURRENT, UNIT_AMPS, 1 },
  { SERVO_FIRST_ID, SERVO_LAST_ID, 1, 4, ZSTR_SERVO_VOLTAGE, UNIT_VOLTS, 1 },
  { SERVO_FIRST_ID, SERVO_LAST_ID, 2, 4, ZSTR_SERVO_TEMPERATURE, UNIT_CELSIUS, 0 },
  { SERVO_FIRST_ID, SERVO_LAST_ID, 3, 4, ZSTR_SERVO_STATUS, UNIT_TEXT, 0 },
  { RAS_ID, RAS_ID, 0, 1, "RAS", UNIT_RAW, 0 },
  { XJT_VERSION_ID, XJT_VERSION_ID, 0, 1, "XJT version", UNIT_RAW, 0 },
#if defined(BETAFLIGHT_SENSORS) || defined(INAV_SENSORS)
  { HOME_DIST_ID, HOME_DIST_ID, 0, 1, ZSTR_DIST, UNIT_METERS, 0 },
#endif
#ifdef INAV_SENSORS
  { INAV_PITCH_ID, INAV_PITCH_ID, 0, 1, ZSTR_PITCH, UNIT_DEGREE, 1 },
  { INAV_ROLL_ID, INAV_ROLL_ID, 0, 1, ZSTR_ROLL, UNIT_DEGREE, 1 },
  { FPV_ID, FPV_ID, 0, 1, ZSTR_HDG, UNIT_DEGREE, 2 },
#endif
  { HALL_EFFECT_ID, HALL_EFFECT_ID, 0, 1, "Hall effect", UNIT_RAW, 0 },
#ifdef BETAFLIGHT_SENSORS
  { BF_PITCH_ID, BF_PITCH_ID, 0, 1, ZSTR_PITCH, UNIT_DEGREE, 1 },
  { BF_ROLL_ID, BF_ROLL_ID, 0, 1, ZSTR_ROLL, UNIT_DEGREE, 1 },
#endif
  { DIY_FIRST_ID, DIY_LAST_ID, 0, 1, "Custom", UNIT_RAW, 0 },
  { 0, 0, 0, 0, nullptr, UNIT_RAW, 0 } // sentinel
};

const SensorInfo* sportGetSensorInfo(uint16_t id, uint8_t subId) {
  for(const SensorInfo *sensorInfo = sportSensorInfos; sensorInfo->firstId; sensorInfo++) {
    if(sensorInfo->firstId <= id && id <= sensorInfo->lastId && subId == sensorInfo->subId) {
      return sensorInfo;
    }
  }
  return nullptr;
}

int sportWriteJsonSensorValue(char* out, const Sensor& sensor) {
  if (sensor.sensorId == FLIGHT_MODE_ID) {
    // max len ~100
    return sportWriteJsonFlightModeArray(out, sensor.value.numeric);
  } else if (sensor.sensorId == GPS_STATE_ID) {
    // max len ~40
    return sportWriteJsonGpsStateArray(out, sensor.value.numeric);
  } else if (sensor.info->unit == UNIT_GPS) {
    return jsonWriteGps(out, sensor.value.gps.longitude, sensor.value.gps.latitude);
  } else {
    return jsonWriteNumber(out, sensor.value.numeric, sensor.info->precision);
  }
}

int sportWriteJsonFlightModeArray(char* out, uint32_t modes) {
  int pos = 0;
  out[pos++] = '[';

  // ones
  uint32_t v = modes;
  uint8_t flag = v % 10;
  if (flag == 1) {
    pos += sprints(out+pos, "\"DISARMED\",");
  } else if(flag == 2) {
    pos += sprints(out+pos, "\"CANT_ARM\",");
  } else if(flag == 5) {
    pos += sprints(out+pos, "\"ARMED\",");
  }

  // tens
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprints(out+pos, "\"ANGLE\",");
  }
  if((flag & 2) == 2) {
    pos += sprints(out+pos, "\"HORIZON\",");
  }
  if((flag & 4) == 4) {
    pos += sprints(out+pos, "\"MANUAL\",");
  }

  // hundreds
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprints(out+pos, "\"HEADING\",");
  }
  if((flag & 2) == 2) {
    pos += sprints(out+pos, "\"NAV_ALTHOLD\",");
  }
  if((flag & 4) == 4) {
    pos += sprints(out+pos, "\"NAV_POSHOLD\",");
  }

  // thousands
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprints(out+pos, "\"NAV_RTH\",");
  }
  if ((flag & 8) == 8) {
    pos += sprints(out+pos, "\"NAV_COURSE_HOLD\",");
  } else if ((flag & 2) == 2) {
    pos += sprints(out+pos, "\"NAV_WP\",");
  } else if ((flag & 4) == 4) {
    pos += sprints(out+pos, "\"HEADFREE\",");
  }

  // ten thousands
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprints(out+pos, "\"FLAPERON\",");
  }
  if ((flag & 4) == 4) {
    pos += sprints(out+pos, "\"FAILSAFE\",");
  } else if ((flag & 2) == 2) {
    pos += sprints(out+pos, "\"AUTO_TUNE\",");
  }

  out[pos-1] = ']';
  return pos;
}

int sportWriteJsonGpsStateArray(char* out, uint32_t state) {
  const char* fix = "WAIT";
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
