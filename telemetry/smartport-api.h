#include "sensors.h"

const SensorInfo* sportGetSensorInfo(uint16_t id, uint8_t subId);
int sportWriteJsonSensorValue(char* out, const Sensor& sensor);
