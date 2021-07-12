#include "platform.h"
#include "json.h"

int jsonWriteGps(char* out, int32_t lon, int32_t lat) {
  char szLon[16], szLat[16];
  dtostrf(lon/10000.0/60.0, 9, 7, szLon);
  dtostrf(lat/10000.0/60.0, 9, 7, szLat);
  // max len 36
  return sprintf(out, "[%s, %s]", szLon, szLat);
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
