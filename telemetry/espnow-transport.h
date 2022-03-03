#ifndef espnow_transport_h
#define espnow_transport_h

/*
 * ESP-NOW interface.
 */

#include "telemetry.h"

void espnowBegin(Telemetry* telemetry);
void espnowStop();
void espnowWrite(const uint8_t* data, int len);

#endif
