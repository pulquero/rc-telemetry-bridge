#define NO_DEBUG

#ifdef DEBUG
  #ifdef ESP_PLATFORM
    #define LOGD(...) Serial.printf(__VA_ARGS__);Serial.println();
    #define LOGE(...) Serial.printf(__VA_ARGS__);Serial.println();
    #define LOGMEM(tag)  LOGD("MEM [%s]: available heap: %d/%d", tag, ESP.getFreeHeap(), ESP.getHeapSize())
  #else
    #define LOGD(...) printf(__VA_ARGS__);printf("\n");
    #define LOGE(...) printf(__VA_ARGS__);printf("\n");
    #define LOGMEM(tag)
  #endif
#else
  #define LOGD(...)
  #define LOGE(...)
  #define LOGMEM(tag)
#endif
