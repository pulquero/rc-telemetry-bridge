#include "bytes.h"

uint16_t read16le(uint8_t*const data) {
  uint16_t result;
  // little endian
  result = data[0];
  result |= data[1] << 8;
  return result;
}

uint32_t read32le(uint8_t*const data) {
  uint32_t result;
  // little endian
  result = data[0];
  result |= data[1] << 8;
  result |= data[2] << 16;
  result |= data[3] << 24;
  return result;
}

uint16_t read16be(uint8_t*const data) {
  uint16_t result;
  // big endian
  result = data[0] << 8;
  result |= data[1];
  return result;
}

uint32_t read24be(uint8_t*const data) {
  uint32_t result;
  // big endian
  result = data[0] << 16;
  result |= data[1] << 8;
  result |= data[2];
  return result;
}

uint32_t read32be(uint8_t*const data) {
  uint32_t result;
  // big endian
  result = data[0] << 24;
  result |= data[1] << 16;
  result |= data[2] << 8;
  result |= data[3];
  return result;
}
