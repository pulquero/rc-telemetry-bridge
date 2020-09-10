#include "protocol.h"
#include "debug.h"

static SPortPacket sportPacket;

bool SPortPacket::add(uint8_t b) {
  // read current packet
  if (b == BYTE_STUFFING) {
    // start of escape sequence
    byteStuffing = true;
  } else {
    if (byteStuffing) {
      // unescape byte
      b ^= STUFFING_MASK;
      byteStuffing = false;
    }

    if (pos < SPORT_BUFFER_SIZE) {
      buffer[pos++] = b;
      if (pos > 1) {
        checksum += b;
        checksum += checksum >> 8;
        checksum &= 0x00FF;
      }
    } else {
      // unexpected state
      LOGD("Ignoring data - SmartPort packet buffer full");
      return false;
    }
  }
  return true;
}

bool SPortPacket::isValid() {
  return checksum == 0xFF;
}

void SPortPacket::clear() {
  pos = 0;
  byteStuffing = false;
  checksum = 0;
}

static int stuffByte(uint8_t*const data, uint8_t b);
static uint16_t read16(uint8_t*const data);
static uint32_t read32(uint8_t*const data);

int sportOnReceive(uint8_t b) {
  static bool waitingForFirstPacket = true;

  if (b == START_STOP) {
    const int packetLen = sportPacket.pos;
    if (waitingForFirstPacket) {
      // start of our first packet
      waitingForFirstPacket = false;
    } else if (packetLen > 0) {
      // end of current packet - start of next
      uint8_t physicalId = sportPacket.buffer[0] & 0x1F;
      if (packetLen == SPORT_DATA_PACKET_LEN) {
        if (sportPacket.isValid()) {
          uint8_t frameId = sportPacket.buffer[1];
          if (frameId == DATA_FRAME) {
            uint16_t sensorId = read16(sportPacket.buffer+2);
            uint32_t sensorData = read32(sportPacket.buffer+4);
            processSensorPacket(physicalId, sensorId, sensorData);
          }
        } else {
          LOGD("SmartPort packet checksum failed");
        }
      } else if (packetLen == SPORT_POLL_PACKET_LEN) {
        processPollPacket(physicalId);
      } else {
        LOGD("Ignoring SmartPort packet of incorrect length: %d", packetLen);
      }
    }
    sportPacket.clear();
    return packetLen; // return +ve value to indicate complete packet
  } else if (!waitingForFirstPacket) {
    // read current packet
    if (!sportPacket.add(b)) {
      // dropped some bytes or something
      waitingForFirstPacket = true;
      sportPacket.clear();
    }
    return -sportPacket.pos; // return -ve value to indicate incomplete packet
  } else {
    return 0;
  }
}

int writeSensorPacket(uint8_t*const out, uint8_t physicalId, uint16_t sensorId, uint32_t sensorData, bool includeStart) {
  int pos = 0;
  if (includeStart) {
    out[pos++] = START_STOP;
  }
  out[pos++] = physicalId;
  const int packetStartPos = pos;
  out[pos++] = DATA_FRAME;
  // little endian
  pos += stuffByte(out+pos, sensorId);
  pos += stuffByte(out+pos, (sensorId>>8));
  pos += stuffByte(out+pos, sensorData);
  pos += stuffByte(out+pos, (sensorData>>8));
  pos += stuffByte(out+pos, (sensorData>>16));
  pos += stuffByte(out+pos, (sensorData>>24));
  uint16_t checksum = 0;
  for (int i=packetStartPos; i<pos; i++) {
    checksum += out[i];
  }
  out[pos++] = 0xFF - ((checksum & 0xFF) + (checksum >> 8));
  out[pos++] = START_STOP;
  return pos;
}

int stuffByte(uint8_t*const data, uint8_t b) {
  switch (b) {
    case START_STOP:
    case BYTE_STUFFING:
      data[0] = BYTE_STUFFING;
      data[1] = b ^ STUFFING_MASK;
      return 2;
    default:
      data[0] = b;
      return 1;
  }
}

uint16_t read16(uint8_t*const data) {
  uint16_t result;
  // little endian
  result = data[0];
  result |= data[1] << 8;
  return result;
}

uint32_t read32(uint8_t*const data) {
  uint32_t result;
  // little endian
  result = data[0];
  result |= data[1] << 8;
  result |= data[2] << 16;
  result |= data[3] << 24;
  return result;
}
