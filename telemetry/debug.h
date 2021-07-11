#include <Arduino.h>

#define NO_DEBUG

#ifdef DEBUG
#define LOGD(...) Serial.printf(__VA_ARGS__);Serial.println();
#define LOGE(...) Serial.printf(__VA_ARGS__);Serial.println();
#define LOGMEM()  LOGD("Available heap: %d/%d", ESP.getFreeHeap(), ESP.getHeapSize())
#else
#define LOGD(...)
#define LOGE(...)
#define LOGMEM()
#endif
