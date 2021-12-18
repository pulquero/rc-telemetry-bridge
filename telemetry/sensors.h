#ifndef sensors_h
#define sensors_h

#include <stdint.h>

#define ZSTR_VFR                       "VFR"
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
#define ZSTR_RB3040_EXTRA_STATE        "RBES"
#define ZSTR_RB3040_CHANNEL1           "CH1A"
#define ZSTR_RB3040_CHANNEL2           "CH2A"
#define ZSTR_RB3040_CHANNEL3           "CH3A"
#define ZSTR_RB3040_CHANNEL4           "CH4A"
#define ZSTR_RB3040_CHANNEL5           "CH5A"
#define ZSTR_RB3040_CHANNEL6           "CH6A"
#define ZSTR_RB3040_CHANNEL7           "CH7A"
#define ZSTR_RB3040_CHANNEL8           "CH8A"
#define ZSTR_FRAME_RATE                "FRat"
#define ZSTR_TOTAL_LATENCY             "TLat"
#define ZSTR_VTX_FREQ                  "VFrq"
#define ZSTR_VTX_PWR                   "VPwr"
#define ZSTR_VTX_CHAN                  "VChn"
#define ZSTR_VTX_BAND                  "VBan"
#define ZSTR_SERVO_CURRENT             "SrvA"
#define ZSTR_SERVO_VOLTAGE             "SrvV"
#define ZSTR_SERVO_TEMPERATURE         "SrvT"
#define ZSTR_SERVO_STATUS              "SrvS"

#define UNKNOWN_SENSOR "Unknown"

enum SensorUnit {
  UNIT_RAW,
  UNIT_VOLTS,
  UNIT_AMPS,
  UNIT_KTS,
  UNIT_METERS_PER_SECOND,
  UNIT_KMH,
  UNIT_METERS,
  UNIT_CELSIUS,
  UNIT_PERCENT,
  UNIT_MAH,
  UNIT_WATTS,
  UNIT_MILLIWATTS,
  UNIT_DB,
  UNIT_RPMS,
  UNIT_G,
  UNIT_DEGREE,
  UNIT_RADIANS,
  UNIT_MILLILITERS,
  UNIT_MILLILITERS_PER_MINUTE,
  UNIT_CELLS,
  UNIT_DATETIME,
  UNIT_GPS,
  UNIT_GPS_LONGITUDE,
  UNIT_GPS_LATITUDE,
  UNIT_BITFIELD,
  UNIT_TEXT
};

enum SensorDataType {
  NATIVE,
  GPS_LONGITUDE,
  GPS_LATITUDE
};

class SensorInfo final {
  public:
  const uint16_t firstId;
  const uint16_t lastId;
  const uint8_t subId;
  const uint8_t subCount;
  const char *const name;
  const SensorUnit unit;
  const uint8_t precision;
  const char* getUnitName() const;
};

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
  void setGpsLongitude(uint32_t sensorData);
  void setGpsLatitude(uint32_t sensorData);
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

  void setValue(uint32_t sensorData, SensorDataType sensorDataType);
};

void processPollPacket(uint8_t physicalId);
void outputSensorPacket(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData);
void processSensorPacket(uint8_t physicalId, uint16_t sensorId, uint8_t subId, uint32_t sensorData, SensorDataType sensorDataType=NATIVE);
int sprints(char* dest, const char* src);

#endif
