#include "telemetry.h"

void protocolBegin(Telemetry* telemetry);
/**
 * Returns the packet size when a complete packet has been received,
 * else minus the size currently received.
 */
int protocolOnReceive(uint8_t b);

const SensorInfo* protocolGetSensorInfo(uint16_t id, uint8_t subId);
/**
 * Returns -1 on error.
 */
int protocolWriteJsonSensorValue(char* out, const Sensor& sensor);
