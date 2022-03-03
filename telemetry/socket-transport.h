#ifndef socket_transport_h
#define socket_transport_h

/*
 * Socket-based interface.
 */

#include "telemetry.h"

void socketBegin(Telemetry* telemetry);
void socketStop();
void socketWrite(const uint8_t* data, int len);
void socketEnsureConnected(uint32_t ms);

#endif
