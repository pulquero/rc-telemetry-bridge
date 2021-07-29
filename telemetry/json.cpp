#include "platform.h"
#include "json.h"
#include "sensors.h"

int jsonWriteGps(char* out, int32_t lon, int32_t lat) {
  char szValue[16];
  int pos = 0;
  out[pos++] = '[';
  dtostrf(lon/10000.0/60.0, 9, 7, szValue);
  pos += sprints(out+pos, szValue);
  out[pos++] = ',';
  out[pos++] = ' ';
  dtostrf(lat/10000.0/60.0, 9, 7, szValue);
  pos += sprints(out+pos, szValue);
  out[pos++] = ']';
  out[pos] = '\0';
  // return strlen (i.e. excluding '\0') - max len 36
  return pos;
}

int jsonWriteNumber(char* out, int32_t value, uint8_t precision) {
  if (precision) {
    double v = value;
    for (int i=0; i<precision; i++) {
      v /= 10.0;
    }
    dtostrf(v, 2+precision, precision, out);
  } else {
    itoa(value, out, 10);
  }
  return strlen(out);
}
