#include "platform.h"
#include "sensors.h"
#include "bytes.h"
#include "crc.h"
#include "ghst.h"
#include "ghst-sensors.h"
#include "debug.h"

#define HEADER_SIZE 2
#define DATA_OFFSET (HEADER_SIZE+1)

// Device (destination) address
#define GHST_ADDR_RADIO                 0x80    // phase 1
#define GHST_ADDR_MODULE_SYM            0x81    // symmetrical, 400k pulses, 400k telemetry
#define GHST_ADDR_MODULE_ASYM           0x88    // asymmetrical, 400k pulses, 115k telemetry
#define GHST_ADDR_FC                    0x82
#define GHST_ADDR_GOGGLES               0x83    // phase 2
#define GHST_ADDR_5G_TXCTRL             0x84    // phase 3
#define GHST_ADDR_5G_TWRSCAN            0x85
#define GHST_ADDR_5G_RLY                0x86

static void ghstProcessPacket(GhstPacket* packet);

static GhstPacket* ghstPacket;

void ghstBegin() {
  ghstPacket = new GhstPacket();
}

void ghstEnd() {
  if (ghstPacket) {
    delete ghstPacket;
    ghstPacket = nullptr;
  }
}

int ghstOnReceive(uint8_t b) {
  static bool waitingForPacket = true;

  if (waitingForPacket && (b&0x80)) {
    waitingForPacket = false;
  }
  if (!waitingForPacket) {
    if (ghstPacket->add(b)) {
      if (ghstPacket->size() == HEADER_SIZE+ghstPacket->frameSize()) {
        if (ghstPacket->isValid()) {
          ghstProcessPacket(ghstPacket);
        } else {
          LOGD("GHST packet CRC failed");
        }
        ghstPacket->clear();
        waitingForPacket = true;
      }
    } else {
      ghstPacket->clear();
      waitingForPacket = true;
    }
  }
  return -ghstPacket->size();
}

void ghstProcessPacket(GhstPacket* packet) {
  const uint8_t addr = packet->deviceAddress();
  const uint8_t frameType = packet->frameType();
  outputGhstSensorPacket(addr, frameType, packet->readData(), packet->frameSize());
  switch (frameType) {
    case GHST_DL_LINK_STAT:
      processSensorPacket(addr, GHOST_ID_RX_RSSI, 0, -packet->read8(0));
      processSensorPacket(addr, GHOST_ID_RX_LQ, 0, packet->read8(1));
      processSensorPacket(addr, GHOST_ID_RX_SNR, 0, packet->read8(2));
      processSensorPacket(addr, GHOST_ID_TX_POWER, 0, packet->read16be(3));
      processSensorPacket(addr, GHOST_ID_FRAME_RATE, 0, packet->read16be(5));
      processSensorPacket(addr, GHOST_ID_TOTAL_LATENCY, 0, packet->read16be(7));
      processSensorPacket(addr, GHOST_ID_RF_MODE, 0, packet->read8(9));
      break;
    case GHST_DL_VTX_STAT:
      processSensorPacket(addr, GHOST_ID_VTX_FREQ, 0, packet->read16be(1));
      processSensorPacket(addr, GHOST_ID_VTX_POWER, 0, packet->read16be(3));
      processSensorPacket(addr, GHOST_ID_VTX_BAND, 0, packet->read8(5));
      processSensorPacket(addr, GHOST_ID_VTX_CHAN, 0, packet->read8(6));
      break;
    case GHST_DL_PACK_STAT:
      processSensorPacket(addr, GHOST_ID_PACK_VOLTS, 0, packet->read16le(0));
      processSensorPacket(addr, GHOST_ID_PACK_AMPS, 0, packet->read16le(2));
      processSensorPacket(addr, GHOST_ID_PACK_MAH, 0, 10*packet->read16le(4));
      break;
    case GHST_DL_GPS_PRIMARY:
      processSensorPacket(addr, GHOST_ID_GPS_LAT, 0, packet->read32le(0)/10);
      processSensorPacket(addr, GHOST_ID_GPS_LONG, 0, packet->read32le(4)/10);
      processSensorPacket(addr, GHOST_ID_GPS_ALT, 0, packet->read16le(8));
      break;
    case GHST_DL_GPS_SECONDARY:
      // cm/s -> km/h
      processSensorPacket(addr, GHOST_ID_GPS_GSPD, 0, (packet->read16le(0)*36 + 50)/100);
      processSensorPacket(addr, GHOST_ID_GPS_HDG, 0, packet->read16le(2)/10);
      processSensorPacket(addr, GHOST_ID_GPS_SATS, 0, packet->read8(4));
      break;
  }
}

int ghstWriteSensorPacket(uint8_t*const out, uint8_t deviceAddr, uint8_t frameType, uint8_t* frameData, int dataLen) {
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

bool GhstPacket::add(uint8_t b) {
  if (pos < GHST_BUFFER_SIZE) {
    buffer[pos++] = b;
    updateChecksum(b);
    return true;
  } else {
    // unexpected state
    LOGD("Ignoring data - GHST packet buffer full");
    return false;
  }
}

void GhstPacket::updateChecksum(uint8_t b) {
  if (pos > HEADER_SIZE && pos < HEADER_SIZE+frameSize()) {
    crc = crcTable[(crc^b) & 0xFF];
  }
}

bool GhstPacket::isValid() {
  return crc == frameCrc();
}

void GhstPacket::clear() {
  pos = 0;
  crc = 0;
}

uint8_t GhstPacket::deviceAddress() {
  return buffer[0];
}

int GhstPacket::frameSize() {
  return (size() >= HEADER_SIZE) ? buffer[1] : 0;
}

uint8_t GhstPacket::frameType() {
  return buffer[2];
}

uint8_t GhstPacket::read8(uint8_t offset) {
  return buffer[DATA_OFFSET+offset];
}

uint16_t GhstPacket::read16be(uint8_t offset) {
  return ::read16be(buffer+DATA_OFFSET+offset);
}

uint16_t GhstPacket::read16le(uint8_t offset) {
  return ::read16le(buffer+DATA_OFFSET+offset);
}

uint32_t GhstPacket::read32le(uint8_t offset) {
  return ::read32le(buffer+DATA_OFFSET+offset);
}

uint8_t* GhstPacket::readData() {
  return &(buffer[DATA_OFFSET]);
}

uint8_t GhstPacket::frameCrc() {
  return buffer[HEADER_SIZE+frameSize()-1];
}

int GhstPacket::size() {
  return pos;
}
