#include <Arduino.h>
#include <string.h>
#include "sensors.h"

int sprints(char* dest, const char* src) {
  strcpy(dest, src);
  return strlen(src);
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
    case UNIT_RADIANS: return "rad";
    default: return "";
  }
}

void Sensor::setValue(uint32_t sensorData, SensorDataType sensorDataType) {
  if (info) {
    if (info->unit == UNIT_GPS) {
      if (sensorDataType == GPS_LONGITUDE) {
        setGpsLongitude(sensorData);
      } else if (sensorDataType == GPS_LATITUDE) {
        setGpsLatitude(sensorData);
      } else {
        setGpsValue(sensorData);
      }
    } else {
      setNumericValue(sensorData);
    }
  } else {
    setNumericValue(sensorData);
  }
  lastUpdated = millis();
}

void Sensor::setGpsValue(uint32_t sensorData) {
  int32_t v = (sensorData & 0x3FFFFFFF); // abs value
  if (sensorData & 0x40000000) {
    // restore sign
    v = -v;
  }
  if (sensorData & 0x80000000) { // is longitude
    setGpsLongitude(v);
  } else {
    setGpsLatitude(v);
  }
}

void Sensor::setGpsLongitude(uint32_t sensorData) {
  if (value.gps.longitude != sensorData) {
    lastChangedValue.gps.longitude = sensorData;
    hasChangedSinceProcessed = true;
  }
  value.gps.longitude = sensorData;
}

void Sensor::setGpsLatitude(uint32_t sensorData) {
  if (value.gps.latitude != sensorData) {
    lastChangedValue.gps.latitude = sensorData;
    hasChangedSinceProcessed = true;
  }
  value.gps.latitude = sensorData;
}

void Sensor::setNumericValue(uint32_t sensorData) {
  const int32_t v = sensorData;
  if (info == nullptr || info->precision <= 1) {
    if (value.numeric != v) {
      lastChangedValue.numeric = v;
      hasChangedSinceProcessed = true;
    }
  } else if(info->precision > 1) {
    int32_t diff = abs(v - lastChangedValue.numeric);
    for (int i=0; i<info->precision; i++) {
      diff /= 10;
    }
    if (diff > 0) {
      lastChangedValue.numeric = v;
      hasChangedSinceProcessed = true;
    }
  }
  value.numeric = v;
}
