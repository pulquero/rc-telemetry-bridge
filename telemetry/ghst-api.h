#include "sensors.h"

const SensorInfo* ghstGetSensorInfo(uint16_t id, uint8_t subId);
int ghstWriteJsonSensorValue(char* out, const Sensor& sensor);
