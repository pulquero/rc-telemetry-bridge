/**
 * Smart port protocol.
 */
#include <Arduino.h>

int sportOnReceive(uint8_t b);
void processPollPacket(uint8_t physicalId);
void processSensorPacket(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData);
int writeSensorPacket(uint8_t*const out, uint8_t physicalId, uint16_t sensorId, uint32_t sensorData, bool includeStart);
