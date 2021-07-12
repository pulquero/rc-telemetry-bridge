#include "sensors.h"

const SensorInfo* crsfGetSensorInfo(uint16_t id, uint8_t subId);
int crsfWriteJsonSensorValue(char* out, const Sensor& sensor);
