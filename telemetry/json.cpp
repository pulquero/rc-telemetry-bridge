#include <Arduino.h>
#include "json.h"

static int writeFlightModeArray(char* out, uint32_t modes);
static int writeGpsStateArray(char* out, uint32_t modes);

int jsonWriteSensorValue(char* value, const Sensor& sensor) {
  int len;
  if (sensor.info) {
    if (sensor.sensorId == FLIGHT_MODE_ID) {
      len = writeFlightModeArray(value, sensor.value.numeric);
    } else if (sensor.sensorId == GPS_STATE_ID) {
      len = writeGpsStateArray(value, sensor.value.numeric);
    } else if (sensor.info->unit == UNIT_GPS) {
      char lon[16], lat[16];
      dtostrf(sensor.value.gps.longitude/10000.0/60.0, 9, 7, lon);
      dtostrf(sensor.value.gps.latitude/10000.0/60.0, 9, 7, lat);
      len = sprintf(value, "[%s, %s]", lon, lat);
    } else {
      if (sensor.info->precision) {
        double v = sensor.value.numeric;
        for (int i=0; i<sensor.info->precision; i++) {
          v /= 10.0;
        }
        dtostrf(v, 2+sensor.info->precision, sensor.info->precision, value);
        len = strlen(value);
      } else {
        itoa(sensor.value.numeric, value, 10);
        len = strlen(value);
      }
    }
  } else {
    // unknown sensor
    itoa(sensor.value.numeric, value, 10);
    len = strlen(value);
  }
  return len;
}

int writeFlightModeArray(char* out, uint32_t modes) {
  int pos = 0;
  out[pos++] = '[';
  uint32_t v = modes;
  uint8_t flag = v % 10;
  if (flag == 1) {
    pos += sprintf(out+pos, "\"DISARMED\",");
  } else if(flag == 2) {
    pos += sprintf(out+pos, "\"CANT_ARM\",");
  } else if(flag == 5) {
    pos += sprintf(out+pos, "\"ARMED\",");
  }
  
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprintf(out+pos, "\"ANGLE\",");
  }
  if((flag & 2) == 2) {
    pos += sprintf(out+pos, "\"HORIZON\",");
  }
  if((flag & 4) == 4) {
    pos += sprintf(out+pos, "\"MANUAL\",");
  }
  
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprintf(out+pos, "\"HEADING\",");
  }
  if((flag & 2) == 2) {
    pos += sprintf(out+pos, "\"NAV_ALTHOLD\",");
  }
  if((flag & 4) == 4) {
    pos += sprintf(out+pos, "\"NAV_POSHOLD\",");
  }
  
  v /= 10;
  flag = v % 10;
  if ((flag & 1) == 1) {
    pos += sprintf(out+pos, "\"NAV_RTH\",");
  }
  if ((flag & 8) == 8) {
    pos += sprintf(out+pos, "\"NAV_CRUISE\",");
  } else if ((flag & 2) == 2) {
    pos += sprintf(out+pos, "\"NAV_WP\",");
  } else if ((flag & 4) == 4) {
    pos += sprintf(out+pos, "\"HEADFREE\",");
  }

  out[pos-1] = ']';
  return pos;
}

int writeGpsStateArray(char* out, uint32_t state) {
  char* fix = "WAIT";
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
