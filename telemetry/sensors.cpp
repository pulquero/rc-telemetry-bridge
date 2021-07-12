#include "sensors.h"

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
