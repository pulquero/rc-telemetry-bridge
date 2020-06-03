#ifndef mqtt_h
#define mqtt_h

/*
 * MQTT interface.
 */

#include "telemetry.h"

void mqttBegin(Telemetry* telemetry);
void mqttStop();
void mqttLoop();
bool mqttPublishSensor(const Sensor& sensor);

#endif
