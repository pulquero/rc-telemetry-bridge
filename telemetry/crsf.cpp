#include "platform.h"
#include "sensors.h"
#include "bytes.h"
#include "crc.h"
#include "crsf.h"
#include "crsf-sensors.h"
#include "debug.h"

#define HEADER_SIZE 2
#define DATA_OFFSET (HEADER_SIZE+1)
#define RADIO_ADDRESS 0xEA

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
static uint32_t toRfMode(uint8_t code);
static uint32_t toRfPower(uint8_t code);
static uint32_t toFlightModeCode(char* fm);

static CrsfPacket* crsfPacket;

void crsfBegin() {
  crsfPacket = new CrsfPacket();
}

void crsfEnd() {
  if (crsfPacket) {
    delete crsfPacket;
    crsfPacket = nullptr;
  }
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
  const uint8_t addr = packet->deviceAddress();
  const uint8_t frameType = packet->frameType();
  outputCrsfSensorPacket(addr, frameType, packet->readData(), packet->frameSize());
  switch (frameType) {
    case CF_VARIO_ID:
      processSensorPacket(addr, CF_VARIO_ID, 0, (int16_t) packet->read16(0));
      break;
    case GPS_ID:
      // 60/1000
      processSensorPacket(addr, GPS_ID, 0, ((int32_t)packet->read32(0))/10*3/5, GPS_LATITUDE);
      processSensorPacket(addr, GPS_ID, 0, ((int32_t)packet->read32(4))/10*3/5, GPS_LONGITUDE);
      processSensorPacket(addr, GPS_ID, 1, packet->read16(8));
      processSensorPacket(addr, GPS_ID, 2, (int16_t) packet->read16(10));
      processSensorPacket(addr, GPS_ID, 3, ((int32_t) packet->read16(12)) - 1000);
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
      processSensorPacket(addr, ATTITUDE_ID, 0, (int16_t) packet->read16(0)/10);
      processSensorPacket(addr, ATTITUDE_ID, 1, (int16_t) packet->read16(2)/10);
      processSensorPacket(addr, ATTITUDE_ID, 2, (int16_t) packet->read16(4)/10);
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
  return ::read16be(buffer+DATA_OFFSET+offset);
}

uint32_t CrsfPacket::read24(uint8_t offset) {
  return ::read24be(buffer+DATA_OFFSET+offset);
}

uint32_t CrsfPacket::read32(uint8_t offset) {
  return ::read32be(buffer+DATA_OFFSET+offset);
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
