#ifndef web_h
#define web_h

/*
 * Web-based interface.
 */

#include "telemetry.h"

void webBegin(Telemetry* telemetry);
void webStop();
void webLoop();
bool webEmitSensor(const Sensor& sensor);

#endif
