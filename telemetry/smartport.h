/**
 * Smart port protocol.
 */
#include <stdint.h>
#include "sensors.h"

#define SPORT_POLL_PACKET_LEN 1
#define SPORT_DATA_PACKET_LEN 9
#define SPORT_BUFFER_SIZE SPORT_DATA_PACKET_LEN

void sportBegin(bool ble);
void sportEnd();
int sportOnReceive(uint8_t b);
int sportWriteSensorPacket(uint8_t*const out, uint8_t physicalId, uint16_t sensorId, uint32_t sensorData, bool includeStart);

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
