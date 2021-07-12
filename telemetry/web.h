#ifndef web_h
#define web_h

/*
 * Web-based interface.
 */

#include <stdint.h>
#include "telemetry.h"

void webBegin(Telemetry* telemetry);
void webStop();
void webLoop(uint32_t ms);
bool webEmitSensor(const Sensor& sensor);

#endif
