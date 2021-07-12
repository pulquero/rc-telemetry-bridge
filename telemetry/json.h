#ifndef json_h
#define json_h

#define JSON_VALUE_BUFFER_SIZE 116

int jsonWriteGps(char* out, int32_t lon, int32_t lat);
int jsonWriteNumber(char* out, int32_t value, uint8_t precision=0);

#endif
