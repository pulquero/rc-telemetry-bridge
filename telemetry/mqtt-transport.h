#ifndef mqtt_transport_h
#define mqtt_transport_h

/*
 * MQTT interface.
 */

#include "telemetry.h"

void mqttBegin(Telemetry* telemetry);
void mqttStop();
void mqttLoop();
bool mqttPublishSensor(const Sensor& sensor);

#endif
