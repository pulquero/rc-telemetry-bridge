/**
 * Smart port protocol.
 */
#include <Arduino.h>
#include "telemetry.h"

#define START_STOP 0x7E
#define BYTE_STUFFING 0x7D
#define STUFFING_MASK 0x20
#define DATA_FRAME 0x10

#define SPORT_POLL_PACKET_LEN 1
#define SPORT_DATA_PACKET_LEN 9
#define SPORT_BUFFER_SIZE SPORT_DATA_PACKET_LEN

void protocolBegin(Telemetry* telemetry);

int sportOnReceive(uint8_t b);
void processPollPacket(uint8_t physicalId);
void processSensorPacket(uint8_t physicalId, uint16_t sensorId, uint32_t sensorData);
int writeSensorPacket(uint8_t*const out, uint8_t physicalId, uint16_t sensorId, uint32_t sensorData, bool includeStart);

class SPortPacket {
  private:
  bool byteStuffing = false;

  protected:
  uint8_t buffer[SPORT_BUFFER_SIZE];
  int pos = 0;
  uint16_t checksum = 0;
  virtual void updateChecksum(uint8_t b);

  public:
  bool add(uint8_t b);
  virtual bool isValid();
  void clear();
  uint8_t physicalId();
  uint8_t frameId();
  uint16_t sensorId();
  uint32_t sensorData();
  int size();
};

class BleSPortPacket final : public SPortPacket {
  void updateChecksum(uint8_t b);
  bool isValid();
};
