#include <stdint.h>

#define GHST_BUFFER_SIZE 32

void ghstBegin();
void ghstEnd();
int ghstOnReceive(uint8_t b);

class GhstPacket final {
  private:
  uint8_t buffer[GHST_BUFFER_SIZE];
  int pos = 0;
  uint8_t crc = 0;
  void updateChecksum(uint8_t b);

  public:
  bool add(uint8_t b);
  bool isValid();
  void clear();
  uint8_t deviceAddress();
  int frameSize();
  uint8_t frameType();
  uint8_t read8(uint8_t offset);
  uint16_t read16be(uint8_t offset);
  uint16_t read16le(uint8_t offset);
  uint32_t read32le(uint8_t offset);
  uint8_t frameCrc();
  int size();
};
