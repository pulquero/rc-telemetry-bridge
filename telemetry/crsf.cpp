#include "platform.h"
#include "sensors.h"
#include "crsf.h"
#include "crsf-sensors.h"
#include "debug.h"

#define HEADER_SIZE 2
#define DATA_OFFSET (HEADER_SIZE+1)
#define RADIO_ADDRESS 0xEA

#define SPORT_PHYSICAL_ID 0x1B

enum CrsfRfMode {
  CRSF_RF_MODE_4_HZ = 0,
  CRSF_RF_MODE_50_HZ = 1,
  CRSF_RF_MODE_150_HZ = 2
};

static const uint32_t rfModeValues[] = {4, 50, 150};

enum CrsfRfPower {
  CRSF_RF_POWER_0_mW = 0,
  CRSF_RF_POWER_10_mW = 1,
  CRSF_RF_POWER_25_mW = 2,
  CRSF_RF_POWER_100_mW = 3,
  CRSF_RF_POWER_500_mW = 4,
  CRSF_RF_POWER_1000_mW = 5,
  CRSF_RF_POWER_2000_mW = 6,
  CRSF_RF_POWER_250_mW = 7
};

static const uint32_t rfPowerValues[] = {0, 10, 25, 100, 500, 1000, 2000, 250};

static void crsfProcessPacket(CrsfPacket* packet);
static uint16_t read16(uint8_t*const data);
static uint32_t read24(uint8_t*const data);
static uint32_t read32(uint8_t*const data);
static uint32_t toRfMode(uint8_t code);
static uint32_t toRfPower(uint8_t code);
static uint32_t toFlightModeCode(char* fm);

static const uint8_t crcTable[] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9, 0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2, 0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92, 0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64, 0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB, 0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

static CrsfPacket* crsfPacket;

void crsfInit() {
  crsfPacket = new CrsfPacket();
}

int crsfOnReceive(uint8_t b) {
  static bool waitingForPacket = true;

  if (waitingForPacket && b == RADIO_ADDRESS) {
    waitingForPacket = false;
  }
  if (!waitingForPacket) {
    if (crsfPacket->add(b)) {
      if (crsfPacket->size() == HEADER_SIZE+crsfPacket->frameSize()) {
        if (crsfPacket->isValid()) {
          crsfProcessPacket(crsfPacket);
        } else {
          LOGD("CRSF packet CRC failed");
        }
        crsfPacket->clear();
        waitingForPacket = true;
      }
    } else {
      crsfPacket->clear();
      waitingForPacket = true;
    }
  }
  return -crsfPacket->size();
}

void crsfProcessPacket(CrsfPacket* packet) {
  uint8_t addr = packet->deviceAddress();
  switch (packet->frameType()) {
    case CF_VARIO_ID:
      processSensorPacket(addr, CF_VARIO_ID, 0, packet->read16(0));
      break;
    case GPS_ID:
      processSensorPacket(addr, GPS_ID, 0, packet->read32(0), GPS_LATITUDE);
      processSensorPacket(addr, GPS_ID, 0, packet->read32(4), GPS_LONGITUDE);
      processSensorPacket(addr, GPS_ID, 1, packet->read16(8));
      processSensorPacket(addr, GPS_ID, 2, packet->read16(10));
      processSensorPacket(addr, GPS_ID, 3, packet->read16(12));
      processSensorPacket(addr, GPS_ID, 4, packet->read8(14));
      break;
    case LINK_ID:
      processSensorPacket(addr, LINK_ID, 0, packet->read8(0));
      processSensorPacket(addr, LINK_ID, 1, packet->read8(1));
      processSensorPacket(addr, LINK_ID, 2, packet->read8(2));
      processSensorPacket(addr, LINK_ID, 3, packet->read8(3));
      processSensorPacket(addr, LINK_ID, 4, packet->read8(4));
      processSensorPacket(addr, LINK_ID, 5, toRfMode(packet->read8(5)));
      processSensorPacket(addr, LINK_ID, 6, toRfPower(packet->read8(6)));
      processSensorPacket(addr, LINK_ID, 7, packet->read8(7));
      processSensorPacket(addr, LINK_ID, 8, packet->read8(8));
      processSensorPacket(addr, LINK_ID, 9, packet->read8(9));
      break;
    case BATTERY_ID:
      processSensorPacket(addr, BATTERY_ID, 0, packet->read16(0));
      processSensorPacket(addr, BATTERY_ID, 1, packet->read16(2));
      processSensorPacket(addr, BATTERY_ID, 2, packet->read24(4));
      processSensorPacket(addr, BATTERY_ID, 3, packet->read8(7));
      break;
    case ATTITUDE_ID:
      processSensorPacket(addr, ATTITUDE_ID, 0, packet->read16(0));
      processSensorPacket(addr, ATTITUDE_ID, 1, packet->read16(2));
      processSensorPacket(addr, ATTITUDE_ID, 2, packet->read16(4));
      break;
    case FLIGHT_MODE_ID:
      processSensorPacket(addr, FLIGHT_MODE_ID, 0, toFlightModeCode(packet->readText()));
      break;
    case RADIO_ID:
      processSensorPacket(addr, RADIO_ID, 0, packet->read32(0));
      break;
  }
}

int crsfWriteSensorPacket(uint8_t*const out, uint8_t deviceAddr, uint8_t frameType, uint8_t* frameData, int dataLen) {
  int pos = 0;
  out[pos++] = deviceAddr;
  out[pos++] = dataLen+2; // frameLength includes frameType and CRC
  out[pos++] = frameType;
  memcpy(out+pos, frameData, dataLen);
  pos += dataLen;
  uint8_t crc = 0;
  for (int i=HEADER_SIZE; i<HEADER_SIZE+1+dataLen; i++) {
    crc = crcTable[(crc^out[i]) & 0xFF];
  }
  out[pos++] = crc;
  return pos;
}

uint16_t read16(uint8_t*const data) {
  uint16_t result;
  // big endian
  result = data[0] << 8;
  result |= data[1];
  return result;
}

uint32_t read24(uint8_t*const data) {
  uint32_t result;
  // big endian
  result = data[0] << 16;
  result |= data[1] << 8;
  result |= data[2];
  return result;
}

uint32_t read32(uint8_t*const data) {
  uint32_t result;
  // big endian
  result = data[0] << 24;
  result |= data[1] << 16;
  result |= data[2] << 8;
  result |= data[3];
  return result;
}

uint32_t toRfMode(uint8_t code) {
  if (code < sizeof(rfModeValues)/sizeof(rfModeValues[0])) {
    return rfModeValues[code];
  } else {
    return -1;
  }
}

uint32_t toRfPower(uint8_t code) {
  if (code < sizeof(rfPowerValues)/sizeof(rfPowerValues[0])) {
    return rfPowerValues[code];
  } else {
    return -1;
  }
}

uint32_t toFlightModeCode(char* fm) {
  if (!strcmp(fm, "OK")) {
    return CRSF_FM_OK;
  } else if (!strcmp(fm, "AIR")) {
    return CRSF_FM_AIR;
  } else if (!strcmp(fm, "ACRO")) {
    return CRSF_FM_ACRO;
  } else if (!strcmp(fm, "!FS!")) {
    return CRSF_FM_FS;
  } else if (!strcmp(fm, "HRST")) {
    return CRSF_FM_HRST;
  } else if (!strcmp(fm, "MANU")) {
    return CRSF_FM_MANU;
  } else if (!strcmp(fm, "RTH")) {
    return CRSF_FM_RTH;
  } else if (!strcmp(fm, "HOLD")) {
    return CRSF_FM_HOLD;
  } else if (!strcmp(fm, "3CRS")) {
    return CRSF_FM_3CRS;
  } else if (!strcmp(fm, "CRS")) {
    return CRSF_FM_CRS;
  } else if (!strcmp(fm, "AH")) {
    return CRSF_FM_AH;
  } else if (!strcmp(fm, "WP")) {
    return CRSF_FM_WP;
  } else if (!strcmp(fm, "ANGL")) {
    return CRSF_FM_ANGL;
  } else if (!strcmp(fm, "HOR")) {
    return CRSF_FM_HOR;
  } else if (!strcmp(fm, "WAIT")) {
    return CRSF_FM_WAIT;
  } else if (!strcmp(fm, "!ERR")) {
    return CRSF_FM_ERR;
  } else {
    return -1;
  }
}

bool CrsfPacket::add(uint8_t b) {
  if (pos < CRSF_BUFFER_SIZE) {
    buffer[pos++] = b;
    updateChecksum(b);
    return true;
  } else {
    // unexpected state
    LOGD("Ignoring data - CRSF packet buffer full");
    return false;
  }
}

void CrsfPacket::updateChecksum(uint8_t b) {
  if (pos > HEADER_SIZE && pos < HEADER_SIZE+frameSize()) {
    crc = crcTable[(crc^b) & 0xFF];
  }
}

bool CrsfPacket::isValid() {
  return crc == frameCrc();
}

void CrsfPacket::clear() {
  pos = 0;
  crc = 0;
}

uint8_t CrsfPacket::deviceAddress() {
  return buffer[0];
}

int CrsfPacket::frameSize() {
  return (size() >= HEADER_SIZE) ? buffer[1] : 0;
}

uint8_t CrsfPacket::frameType() {
  return buffer[2];
}

uint8_t CrsfPacket::read8(uint8_t offset) {
  return buffer[DATA_OFFSET+offset];
}

uint16_t CrsfPacket::read16(uint8_t offset) {
  return ::read16(buffer+DATA_OFFSET+offset);
}

uint32_t CrsfPacket::read24(uint8_t offset) {
  return ::read24(buffer+DATA_OFFSET+offset);
}

uint32_t CrsfPacket::read32(uint8_t offset) {
  return ::read32(buffer+DATA_OFFSET+offset);
}

uint8_t* CrsfPacket::readData() {
  return &(buffer[DATA_OFFSET]);
}

char* CrsfPacket::readText() {
  return (char*) &(buffer[DATA_OFFSET]);
}

uint8_t CrsfPacket::frameCrc() {
  return buffer[HEADER_SIZE+frameSize()-1];
}

int CrsfPacket::size() {
  return pos;
}
