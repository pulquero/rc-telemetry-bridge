#include <Arduino.h>

#define BETAFLIGHT_SENSORS
#define INAV_SENSORS

#define ALT_FIRST_ID              0x0100
#define ALT_LAST_ID               0x010F
#define VARIO_FIRST_ID            0x0110
#define VARIO_LAST_ID             0x011F
#define CURR_FIRST_ID             0x0200
#define CURR_LAST_ID              0x020F
#define VFAS_FIRST_ID             0x0210
#define VFAS_LAST_ID              0x021F
#define CELLS_FIRST_ID            0x0300
#define CELLS_LAST_ID             0x030F
#if defined(BETAFLIGHT_SENSORS) || defined(INAV_SENSORS)
#define FLIGHT_MODE_ID            0x0400
#endif
#define T1_FIRST_ID               0x0400
#define T1_LAST_ID                0x040F
#if defined(BETAFLIGHT_SENSORS) || defined(INAV_SENSORS)
#define GPS_STATE_ID              0x0410
#endif
#define T2_FIRST_ID               0x0410
#define T2_LAST_ID                0x041F
#if defined(BETAFLIGHT_SENSORS) || defined(INAV_SENSORS)
#define HOME_DIST_ID              0x0420
#endif
#ifdef INAV_SENSORS
#define INAV_PITCH_ID             0x0430
#define INAV_ROLL_ID              0x0440
#define FPV_ID                    0x0450
#endif
#define RPM_FIRST_ID              0x0500
#define RPM_LAST_ID               0x050F
#define FUEL_FIRST_ID             0x0600
#define FUEL_LAST_ID              0x060F
#define ACCX_FIRST_ID             0x0700
#define ACCX_LAST_ID              0x070F
#define ACCY_FIRST_ID             0x0710
#define ACCY_LAST_ID              0x071F
#define ACCZ_FIRST_ID             0x0720
#define ACCZ_LAST_ID              0x072F
#define GPS_LONG_LATI_FIRST_ID    0x0800
#define GPS_LONG_LATI_LAST_ID     0x080F
#define GPS_ALT_FIRST_ID          0x0820
#define GPS_ALT_LAST_ID           0x082F
#define GPS_SPEED_FIRST_ID        0x0830
#define GPS_SPEED_LAST_ID         0x083F
#define GPS_COURS_FIRST_ID        0x0840
#define GPS_COURS_LAST_ID         0x084F
#define GPS_TIME_DATE_FIRST_ID    0x0850
#define GPS_TIME_DATE_LAST_ID     0x085F
#define A3_FIRST_ID               0x0900
#define A3_LAST_ID                0x090F
#define A4_FIRST_ID               0x0910
#define A4_LAST_ID                0x091F
#define AIR_SPEED_FIRST_ID        0x0A00
#define AIR_SPEED_LAST_ID         0x0A0F
#define RBOX_BATT1_FIRST_ID       0x0B00
#define RBOX_BATT1_LAST_ID        0x0B0F
#define RBOX_BATT2_FIRST_ID       0x0B10
#define RBOX_BATT2_LAST_ID        0x0B1F
#define RBOX_STATE_FIRST_ID       0x0B20
#define RBOX_STATE_LAST_ID        0x0B2F
#define RBOX_CNSP_FIRST_ID        0x0B30
#define RBOX_CNSP_LAST_ID         0x0B3F
#define SD1_FIRST_ID              0x0B40
#define SD1_LAST_ID               0x0B4F
#define ESC_POWER_FIRST_ID        0x0B50
#define ESC_POWER_LAST_ID         0x0B5F
#define ESC_RPM_CONS_FIRST_ID     0x0B60
#define ESC_RPM_CONS_LAST_ID      0x0B6f
#define ESC_TEMPERATURE_FIRST_ID  0x0B70
#define ESC_TEMPERATURE_LAST_ID   0x0B7f
#define X8R_FIRST_ID              0x0c20
#define X8R_LAST_ID               0x0c2F
#define S6R_FIRST_ID              0x0c30
#define S6R_LAST_ID               0x0c3F
#define GASSUIT_TEMP1_FIRST_ID    0x0D00
#define GASSUIT_TEMP1_LAST_ID     0x0D0F
#define GASSUIT_TEMP2_FIRST_ID    0x0D10
#define GASSUIT_TEMP2_LAST_ID     0x0D1F
#define GASSUIT_SPEED_FIRST_ID    0x0D20
#define GASSUIT_SPEED_LAST_ID     0x0D2F
#define GASSUIT_RES_VOL_FIRST_ID  0x0D30
#define GASSUIT_RES_VOL_LAST_ID   0x0D3F
#define GASSUIT_RES_PERC_FIRST_ID 0x0D40
#define GASSUIT_RES_PERC_LAST_ID  0x0D4F
#define GASSUIT_FLOW_FIRST_ID     0x0D50
#define GASSUIT_FLOW_LAST_ID      0x0D5F
#define GASSUIT_MAX_FLOW_FIRST_ID 0x0D60
#define GASSUIT_MAX_FLOW_LAST_ID  0x0D6f
#define GASSUIT_AVG_FLOW_FIRST_ID 0x0D70
#define GASSUIT_AVG_FLOW_LAST_ID  0x0D7f
#define SBEC_POWER_FIRST_ID       0x0E50
#define SBEC_POWER_LAST_ID        0x0E5F
#define DIY_FIRST_ID              0x5100
#define HALL_EFFECT_ID            0x5131
#ifdef BETAFLIGHT_SENSORS
#define BF_PITCH_ID               0x5230
#define BF_ROLL_ID                0x5240
#endif
#define DIY_LAST_ID               0x52FF
#define DIY_STREAM_FIRST_ID       0x5000
#define DIY_STREAM_LAST_ID        0x50FF
#define FACT_TEST_ID              0xF000
#define RSSI_ID                   0xF101
#define ADC1_ID                   0xF102
#define ADC2_ID                   0xF103
#define BATT_ID                   0xF104
#define RAS_ID                    0xF105
#define XJT_VERSION_ID            0xF106
#define R9_PWR_ID                 0xF107
#define SP2UART_A_ID              0xFD00
#define SP2UART_B_ID              0xFD01
#define FUEL_QTY_FIRST_ID         0x0A10
#define FUEL_QTY_LAST_ID          0x0A1F

enum SensorUnit {
  UNIT_RAW,
  UNIT_VOLTS,
  UNIT_AMPS,
  UNIT_KTS,
  UNIT_METERS_PER_SECOND,
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
  UNIT_MILLILITERS,
  UNIT_MILLILITERS_PER_MINUTE,
  UNIT_CELLS,
  UNIT_DATETIME,
  UNIT_GPS,
  UNIT_BITFIELD,
  UNIT_TEXT
};

class SensorInfo final {
  public:
  const uint16_t firstId;
  const uint16_t lastId;
  const uint8_t subId;
  const char *const name;
  const SensorUnit unit;
  const uint8_t precision;
  const char* getUnitName() const;
};

const SensorInfo* getSensorInfo(uint16_t id, uint8_t subId);
