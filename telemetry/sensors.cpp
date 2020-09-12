
#include "sensors.h"

#define ZSTR_RSSI                      "RSSI"
#define ZSTR_R9PW                      "R9PW"
#define ZSTR_RAS                       "SWR"
#define ZSTR_A1                        "A1"
#define ZSTR_A2                        "A2"
#define ZSTR_A3                        "A3"
#define ZSTR_A4                        "A4"
#define ZSTR_BATT                      "RxBt"
#define ZSTR_ALT                       "Alt"
#define ZSTR_TEMP1                     "Tmp1"
#define ZSTR_TEMP2                     "Tmp2"
#define ZSTR_TEMP3                     "Tmp3"
#define ZSTR_TEMP4                     "Tmp4"
#define ZSTR_RPM2                      "RPM2"
#define ZSTR_PRES                      "Pres"
#define ZSTR_ODO1                      "Odo1"
#define ZSTR_ODO2                      "Odo2"
#define ZSTR_TXV                       "TX_V"
#define ZSTR_CURR_SERVO1               "CSv1"
#define ZSTR_CURR_SERVO2               "CSv2"
#define ZSTR_CURR_SERVO3               "CSv3"
#define ZSTR_CURR_SERVO4               "CSv4"
#define ZSTR_DIST                      "Dist"
#define ZSTR_ARM                       "Arm"
#define ZSTR_C50                       "C50"
#define ZSTR_C200                      "C200"
#define ZSTR_RPM                       "RPM"
#define ZSTR_FUEL                      "Fuel"
#define ZSTR_VSPD                      "VSpd"
#define ZSTR_ACCX                      "AccX"
#define ZSTR_ACCY                      "AccY"
#define ZSTR_ACCZ                      "AccZ"
#define ZSTR_GYROX                     "GyrX"
#define ZSTR_GYROY                     "GyrY"
#define ZSTR_GYROZ                     "GyrZ"
#define ZSTR_CURR                      "Current"
#define ZSTR_CAPACITY                  "Capa"
#define ZSTR_VFAS                      "VFAS"
#define ZSTR_BATT_PERCENT              "Bat%"
#define ZSTR_ASPD                      "Air speed"
#define ZSTR_GSPD                      "GPS speed"
#define ZSTR_HDG                       "Heading"
#define ZSTR_SATELLITES                "Sats"
#define ZSTR_CELLS                     "Cells"
#define ZSTR_GPSALT                    "GPS Alt"
#define ZSTR_GPSDATETIME               "Date"
#define ZSTR_GPS                       "GPS"
#define ZSTR_BATT1_VOLTAGE             "RB1V"
#define ZSTR_BATT2_VOLTAGE             "RB2V"
#define ZSTR_BATT1_CURRENT             "RB1A"
#define ZSTR_BATT2_CURRENT             "RB2A"
#define ZSTR_BATT1_CONSUMPTION         "RB1C"
#define ZSTR_BATT2_CONSUMPTION         "RB2C"
#define ZSTR_BATT1_TEMP                "RB1T"
#define ZSTR_BATT2_TEMP                "RB2T"
#define ZSTR_RB_STATE                  "RBS"
#define ZSTR_CHANS_STATE               "RBCS"
#define ZSTR_RX_RSSI1                  "1RSS"
#define ZSTR_RX_RSSI2                  "2RSS"
#define ZSTR_RX_QUALITY                "RQly"
#define ZSTR_RX_SNR                    "RSNR"
#define ZSTR_RX_NOISE                  "RNse"
#define ZSTR_ANTENNA                   "ANT"
#define ZSTR_RF_MODE                   "RFMD"
#define ZSTR_TX_POWER                  "TPWR"
#define ZSTR_TX_RSSI                   "TRSS"
#define ZSTR_TX_QUALITY                "TQly"
#define ZSTR_TX_SNR                    "TSNR"
#define ZSTR_TX_NOISE                  "TNse"
#define ZSTR_PITCH                     "Pitch"
#define ZSTR_ROLL                      "Roll"
#define ZSTR_YAW                       "Yaw"
#define ZSTR_FLIGHT_MODE               "FM"
#define ZSTR_THROTTLE                  "Thr"
#define ZSTR_QOS_A                     "FdeA"
#define ZSTR_QOS_B                     "FdeB"
#define ZSTR_QOS_L                     "FdeL"
#define ZSTR_QOS_R                     "FdeR"
#define ZSTR_QOS_F                     "FLss"
#define ZSTR_QOS_H                     "Hold"
#define ZSTR_BIND                      "BIND"
#define ZSTR_LAP_NUMBER                "Lap "
#define ZSTR_GATE_NUMBER               "Gate"
#define ZSTR_LAP_TIME                  "LapT"
#define ZSTR_GATE_TIME                 "GteT"
#define ZSTR_ESC_VOLTAGE               "EscV"
#define ZSTR_ESC_CURRENT               "EscA"
#define ZSTR_ESC_RPM                   "EscR"
#define ZSTR_ESC_CONSUMPTION           "EscC"
#define ZSTR_ESC_TEMP                  "EscT"
#define ZSTR_SD1_CHANNEL               "Chan"
#define ZSTR_GASSUIT_TEMP1             "GTp1"
#define ZSTR_GASSUIT_TEMP2             "GTp2"
#define ZSTR_GASSUIT_RPM               "GRPM"
#define ZSTR_GASSUIT_FLOW              "GFlo"
#define ZSTR_GASSUIT_CONS              "GFue"
#define ZSTR_GASSUIT_RES_VOL           "GRVl"
#define ZSTR_GASSUIT_RES_PERC          "GRPc"
#define ZSTR_GASSUIT_MAX_FLOW          "GMFl"
#define ZSTR_GASSUIT_AVG_FLOW          "GAFl"
#define ZSTR_SBEC_VOLTAGE              "BecV"
#define ZSTR_SBEC_CURRENT              "BecA"

static const SensorInfo sensorInfos[] = {
  { RSSI_ID, RSSI_ID, 0, ZSTR_RSSI, UNIT_DB, 0 },
  { ADC1_ID, ADC1_ID, 0, ZSTR_A1, UNIT_VOLTS, 1 },
  { ADC2_ID, ADC2_ID, 0, ZSTR_A2, UNIT_VOLTS, 1 },
  { A3_FIRST_ID, A3_LAST_ID, 0, ZSTR_A3, UNIT_VOLTS, 2 },
  { A4_FIRST_ID, A4_LAST_ID, 0, ZSTR_A4, UNIT_VOLTS, 2 },
  { BATT_ID, BATT_ID, 0, ZSTR_BATT, UNIT_VOLTS, 1 },
  { R9_PWR_ID, R9_PWR_ID, 0, ZSTR_R9PW, UNIT_MILLIWATTS, 0 },
  { T1_FIRST_ID, T1_LAST_ID, 0, ZSTR_TEMP1, UNIT_CELSIUS, 0 },
  { T2_FIRST_ID, T2_LAST_ID, 0, ZSTR_TEMP2, UNIT_CELSIUS, 0 },
  { RPM_FIRST_ID, RPM_LAST_ID, 0, ZSTR_RPM, UNIT_RPMS, 0 },
  { FUEL_FIRST_ID, FUEL_LAST_ID, 0, ZSTR_FUEL, UNIT_MAH, 0 },
  { ALT_FIRST_ID, ALT_LAST_ID, 0, ZSTR_ALT, UNIT_METERS, 2 },
  { VARIO_FIRST_ID, VARIO_LAST_ID, 0, ZSTR_VSPD, UNIT_METERS_PER_SECOND, 2 },
  { ACCX_FIRST_ID, ACCX_LAST_ID, 0, ZSTR_ACCX, UNIT_G, 2 },
  { ACCY_FIRST_ID, ACCY_LAST_ID, 0, ZSTR_ACCY, UNIT_G, 2 },
  { ACCZ_FIRST_ID, ACCZ_LAST_ID, 0, ZSTR_ACCZ, UNIT_G, 2 },
  { CURR_FIRST_ID, CURR_LAST_ID, 0, ZSTR_CURR, UNIT_AMPS, 1 },
  { VFAS_FIRST_ID, VFAS_LAST_ID, 0, ZSTR_VFAS, UNIT_VOLTS, 2 },
  { AIR_SPEED_FIRST_ID, AIR_SPEED_LAST_ID, 0, ZSTR_ASPD, UNIT_KTS, 1 },
  { GPS_SPEED_FIRST_ID, GPS_SPEED_LAST_ID, 0, ZSTR_GSPD, UNIT_KTS, 3 },
  { CELLS_FIRST_ID, CELLS_LAST_ID, 0, ZSTR_CELLS, UNIT_CELLS, 2 },
  { GPS_ALT_FIRST_ID, GPS_ALT_LAST_ID, 0, ZSTR_GPSALT, UNIT_METERS, 2 },
  { GPS_TIME_DATE_FIRST_ID, GPS_TIME_DATE_LAST_ID, 0, ZSTR_GPSDATETIME, UNIT_DATETIME, 0 },
  { GPS_LONG_LATI_FIRST_ID, GPS_LONG_LATI_LAST_ID, 0, ZSTR_GPS, UNIT_GPS, 0 },
  { FUEL_QTY_FIRST_ID, FUEL_QTY_LAST_ID, 0, ZSTR_FUEL, UNIT_MILLILITERS, 2 },
  { GPS_COURS_FIRST_ID, GPS_COURS_LAST_ID, 0, ZSTR_HDG, UNIT_DEGREE, 2 },
  { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 0, ZSTR_BATT1_VOLTAGE, UNIT_VOLTS, 3 },
  { RBOX_BATT2_FIRST_ID, RBOX_BATT2_LAST_ID, 0, ZSTR_BATT2_VOLTAGE, UNIT_VOLTS, 3 },
  { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 1, ZSTR_BATT1_CURRENT, UNIT_AMPS, 2 },
  { RBOX_BATT2_FIRST_ID, RBOX_BATT2_LAST_ID, 1, ZSTR_BATT2_CURRENT, UNIT_AMPS, 2 },
  { RBOX_CNSP_FIRST_ID, RBOX_CNSP_LAST_ID, 0, ZSTR_BATT1_CONSUMPTION, UNIT_MAH, 0 },
  { RBOX_CNSP_FIRST_ID, RBOX_CNSP_LAST_ID, 1, ZSTR_BATT2_CONSUMPTION, UNIT_MAH, 0 },
  { RBOX_STATE_FIRST_ID, RBOX_STATE_LAST_ID, 0, ZSTR_CHANS_STATE, UNIT_BITFIELD, 0 },
  { RBOX_STATE_FIRST_ID, RBOX_STATE_LAST_ID, 1, ZSTR_RB_STATE, UNIT_BITFIELD, 0 },
  { SD1_FIRST_ID, SD1_LAST_ID, 0, ZSTR_SD1_CHANNEL, UNIT_RAW, 0 },
  { ESC_POWER_FIRST_ID, ESC_POWER_LAST_ID, 0, ZSTR_ESC_VOLTAGE, UNIT_VOLTS, 2 },
  { ESC_POWER_FIRST_ID, ESC_POWER_LAST_ID, 1, ZSTR_ESC_CURRENT, UNIT_AMPS, 2 },
  { ESC_RPM_CONS_FIRST_ID, ESC_RPM_CONS_LAST_ID, 0, ZSTR_ESC_RPM, UNIT_RPMS, 0 },
  { ESC_RPM_CONS_FIRST_ID, ESC_RPM_CONS_LAST_ID, 1, ZSTR_ESC_CONSUMPTION, UNIT_MAH, 0 },
  { ESC_TEMPERATURE_FIRST_ID, ESC_TEMPERATURE_LAST_ID, 0, ZSTR_ESC_TEMP, UNIT_CELSIUS, 0 },
  { GASSUIT_TEMP1_FIRST_ID, GASSUIT_TEMP1_LAST_ID, 0, ZSTR_GASSUIT_TEMP1, UNIT_CELSIUS, 0 },
  { GASSUIT_TEMP2_FIRST_ID, GASSUIT_TEMP2_LAST_ID, 0, ZSTR_GASSUIT_TEMP2, UNIT_CELSIUS, 0 },
  { GASSUIT_SPEED_FIRST_ID, GASSUIT_SPEED_LAST_ID, 0, ZSTR_GASSUIT_RPM, UNIT_RPMS, 0 },
  { GASSUIT_RES_VOL_FIRST_ID, GASSUIT_RES_VOL_LAST_ID, 0, ZSTR_GASSUIT_RES_VOL, UNIT_MILLILITERS, 0 },
  { GASSUIT_RES_PERC_FIRST_ID, GASSUIT_RES_PERC_LAST_ID, 0, ZSTR_GASSUIT_RES_PERC, UNIT_PERCENT, 0 },
  { GASSUIT_FLOW_FIRST_ID, GASSUIT_FLOW_LAST_ID, 0, ZSTR_GASSUIT_FLOW, UNIT_MILLILITERS_PER_MINUTE, 0 },
  { GASSUIT_MAX_FLOW_FIRST_ID, GASSUIT_MAX_FLOW_LAST_ID, 0, ZSTR_GASSUIT_MAX_FLOW, UNIT_MILLILITERS_PER_MINUTE, 0 },
  { GASSUIT_AVG_FLOW_FIRST_ID, GASSUIT_AVG_FLOW_LAST_ID, 0, ZSTR_GASSUIT_AVG_FLOW, UNIT_MILLILITERS_PER_MINUTE, 0 },
  { SBEC_POWER_FIRST_ID, SBEC_POWER_LAST_ID, 0, ZSTR_SBEC_VOLTAGE, UNIT_VOLTS, 2 },
  { SBEC_POWER_FIRST_ID, SBEC_POWER_LAST_ID, 1, ZSTR_SBEC_CURRENT, UNIT_AMPS, 2 },
  { XJT_VERSION_ID, XJT_VERSION_ID, 0, "XJT version", UNIT_RAW, 0 },
#if defined(BETAFLIGHT_SENSORS) || defined(INAV_SENSORS)
  { HOME_DIST_ID, HOME_DIST_ID, 0, ZSTR_DIST, UNIT_METERS, 0 },
#endif
#ifdef INAV_SENSORS
  { INAV_PITCH_ID, INAV_PITCH_ID, 0, ZSTR_PITCH, UNIT_DEGREE, 1 },
  { INAV_ROLL_ID, INAV_ROLL_ID, 0, ZSTR_ROLL, UNIT_DEGREE, 1 },
  { FPV_ID, FPV_ID, 0, ZSTR_HDG, UNIT_DEGREE, 2 },
#endif
  { HALL_EFFECT_ID, HALL_EFFECT_ID, 0, "Hall effect", UNIT_RAW, 0 },
#ifdef BETAFLIGHT_SENSORS
  { BF_PITCH_ID, BF_PITCH_ID, 0, ZSTR_PITCH, UNIT_DEGREE, 1 },
  { BF_ROLL_ID, BF_ROLL_ID, 0, ZSTR_ROLL, UNIT_DEGREE, 1 },
#endif
  { DIY_FIRST_ID, DIY_LAST_ID, 0, "Custom", UNIT_RAW, 0 },
  { 0, 0, 0, nullptr, UNIT_RAW, 0 } // sentinel
};

const SensorInfo* getSensorInfo(uint16_t id, uint8_t subId) {
  for(const SensorInfo *sensorInfo = sensorInfos; sensorInfo->firstId; sensorInfo++) {
    if(sensorInfo->firstId <= id && id <= sensorInfo->lastId && subId == sensorInfo->subId) {
      return sensorInfo;
    }
  }
  return nullptr;
}

const char* SensorInfo::getUnitName() const {
  switch (unit) {
    case UNIT_VOLTS: return "V";
    case UNIT_AMPS: return "A";
    case UNIT_KTS: return "kt";
    case UNIT_METERS_PER_SECOND: return "m/s";
    case UNIT_METERS: return "m";
    case UNIT_CELSIUS: return "°C";
    case UNIT_PERCENT: return "%";
    case UNIT_MAH: return "mAh";
    case UNIT_WATTS: return "W";
    case UNIT_MILLIWATTS: return "mW";
    case UNIT_RPMS: return "rpm";
    case UNIT_DB: return "dB";
    case UNIT_G: return "g";
    case UNIT_DEGREE: return "°";
    default: return "";
  }
}
