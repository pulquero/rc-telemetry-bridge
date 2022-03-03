#include "protocol.h"
#include "smartport.h"
#include "smartport-api.h"
#include "crsf.h"
#include "crsf-api.h"
#include "ghst.h"
#include "ghst-api.h"
#include "json.h"

static Telemetry* _telemetry;

void protocolBegin(Telemetry* telemetry) {
  _telemetry = telemetry;
  switch (_telemetry->config.input.protocol) {
  case PROTOCOL_CRSF:
      crsfBegin();
      break;
  case PROTOCOL_GHST:
      ghstBegin();
      break;
  case PROTOCOL_SMART_PORT:
      if (telemetry->config.input.source == SOURCE_BLE) {
        sportBegin(true);
      } else {
        sportBegin(false);
      }
      break;
  default:
      break;
  }
}

void protocolEnd() {
  switch (_telemetry->config.input.protocol) {
  case PROTOCOL_CRSF:
      crsfEnd();
      break;
  case PROTOCOL_GHST:
      ghstEnd();
      break;
  case PROTOCOL_SMART_PORT:
      sportEnd();
      break;
  default:
      break;
  }
}

int protocolOnReceive(uint8_t b) {
  int packetSize;
  switch (_telemetry->config.input.protocol) {
  case PROTOCOL_CRSF:
      packetSize = crsfOnReceive(b);
      break;
  case PROTOCOL_GHST:
      packetSize = ghstOnReceive(b);
      break;
  case PROTOCOL_SMART_PORT:
      packetSize = sportOnReceive(b);
      break;
  default:
      packetSize = 0;
      break;
  }
  return packetSize;
}

const SensorInfo* protocolGetSensorInfo(uint16_t id, uint8_t subId) {
  const SensorInfo* info;
  switch (_telemetry->config.input.protocol) {
  case PROTOCOL_CRSF:
      info = crsfGetSensorInfo(id, subId);
      break;
  case PROTOCOL_GHST:
      info = ghstGetSensorInfo(id, subId);
      break;
  case PROTOCOL_SMART_PORT:
      info = sportGetSensorInfo(id, subId);
      break;
  default:
      info = nullptr;
      break;
  }
  return info;
}

int protocolWriteJsonSensorValue(char* out, const Sensor& sensor) {
  int bytesWritten;
  if (sensor.info) {
    switch (_telemetry->config.input.protocol) {
    case PROTOCOL_CRSF:
        bytesWritten = crsfWriteJsonSensorValue(out, sensor);
        break;
    case PROTOCOL_GHST:
        bytesWritten = ghstWriteJsonSensorValue(out, sensor);
        break;
    case PROTOCOL_SMART_PORT:
        bytesWritten = sportWriteJsonSensorValue(out, sensor);
        break;
    default:
        bytesWritten = jsonWriteNumber(out, sensor.value.numeric);
        break;
    }
  } else {
    // unknown sensor (max len 11)
    bytesWritten = jsonWriteNumber(out, sensor.value.numeric);
  }
  return bytesWritten;
}
