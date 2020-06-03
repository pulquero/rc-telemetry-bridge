#ifndef json_h
#define json_h

#include "telemetry.h"

#define JSON_VALUE_BUFFER_SIZE 128

int jsonWriteSensorValue(char* value, const Sensor& sensor);

#endif
