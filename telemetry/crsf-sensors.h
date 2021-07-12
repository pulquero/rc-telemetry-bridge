#define GPS_ID                         0x02
#define CF_VARIO_ID                    0x07
#define BATTERY_ID                     0x08
#define LINK_ID                        0x14
#define CHANNELS_ID                    0x16
#define ATTITUDE_ID                    0x1E
#define FLIGHT_MODE_ID                 0x21
#define PING_DEVICES_ID                0x28
#define DEVICE_INFO_ID                 0x29
#define REQUEST_SETTINGS_ID            0x2A
#define COMMAND_ID                     0x32
#define RADIO_ID                       0x3A

enum CrsfFlightMode {
  CRSF_FM_OK,
  CRSF_FM_AIR,
  CRSF_FM_ACRO,
  CRSF_FM_FS,
  CRSF_FM_HRST,
  CRSF_FM_MANU,
  CRSF_FM_RTH,
  CRSF_FM_HOLD,
  CRSF_FM_3CRS,
  CRSF_FM_CRS,
  CRSF_FM_AH,
  CRSF_FM_WP,
  CRSF_FM_ANGL,
  CRSF_FM_HOR,
  CRSF_FM_WAIT,
  CRSF_FM_ERR
};

static const char*const flightModeValues[] = {
  "OK",
  "AIR",
  "ACRO",
  "!FS!",
  "HRST",
  "MANU",
  "RTH",
  "HOLD",
  "3CRS",
  "CRS",
  "AH",
  "WP",
  "ANGL",
  "HOR",
  "WAIT",
  "!ERR"
};
