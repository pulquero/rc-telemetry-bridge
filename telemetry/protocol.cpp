#include "protocol.h"
#include "smartport.h"
#include "smartport-api.h"
#include "crsf.h"
#include "crsf-api.h"
#include "json.h"

static Telemetry* _telemetry;

void protocolBegin(Telemetry* telemetry) {
  _telemetry = telemetry;
  if (telemetry->config.input.protocol == PROTOCOL_CRSF) {
    crsfInit();
  } else {
    if (telemetry->config.input.source == SOURCE_BLE) {
      sportInit(true);
    } else {
      sportInit(false);
    }
  }
}

int protocolOnReceive(uint8_t b) {
  if (_telemetry->config.input.protocol == PROTOCOL_CRSF) {
    return crsfOnReceive(b);
  } else {
    return sportOnReceive(b);
  }
}

const SensorInfo* protocolGetSensorInfo(uint16_t id, uint8_t subId) {
  if (_telemetry->config.input.protocol == PROTOCOL_CRSF) {
    return crsfGetSensorInfo(id, subId);
  } else {
    return sportGetSensorInfo(id, subId);
  }
}

int protocolWriteJsonSensorValue(char* out, const Sensor& sensor) {
  if (sensor.info) {
    if (_telemetry->config.input.protocol == PROTOCOL_CRSF) {
      return crsfWriteJsonSensorValue(out, sensor);
    } else {
      return sportWriteJsonSensorValue(out, sensor);
    }
  } else {
    // unknown sensor (max len 11)
    return jsonWriteNumber(out, sensor.value.numeric);
  }
}
