/**
 * Smart port protocol.
 */
#include <Arduino.h>

#define START_STOP 0x7E
#define BYTE_STUFFING 0x7D
#define STUFFING_MASK 0x20
#define DATA_FRAME 0x10

#define SPORT_POLL_PACKET_LEN 1
#define SPORT_DATA_PACKET_LEN 9
#define SPORT_BUFFER_SIZE SPORT_DATA_PACKET_LEN

int sportOnReceive(uint8_t b);
void processPollPacket(uint8_t physicalId);
void processSensorPacket(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData);
int writeSensorPacket(uint8_t*const out, uint8_t physicalId, uint16_t sensorId, uint32_t sensorData, bool includeStart);

class SPortPacket final {
  private:
  bool byteStuffing = false;
  uint16_t checksum = 0;
  public:
  static bool isStartStop(uint8_t b) { return b == START_STOP; };

  uint8_t buffer[SPORT_BUFFER_SIZE];
  int pos = 0;

  bool add(uint8_t b);
  bool isValid();
  void clear();
};
