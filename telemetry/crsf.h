#include <stdint.h>

#define CRSF_BUFFER_SIZE 64

void crsfBegin();
void crsfEnd();
int crsfOnReceive(uint8_t b);

class CrsfPacket final {
  private:
  uint8_t buffer[CRSF_BUFFER_SIZE];
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
  uint16_t read16(uint8_t offset);
  uint32_t read24(uint8_t offset);
  uint32_t read32(uint8_t offset);
  uint8_t* readData();
  char* readText();
  uint8_t frameCrc();
  int size();
};
