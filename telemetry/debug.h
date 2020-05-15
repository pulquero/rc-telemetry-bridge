#define NO_DEBUG

#ifdef DEBUG
#define LOGD(...) Serial.printf(__VA_ARGS__);Serial.println();
#define LOGE(...) Serial.printf(__VA_ARGS__);Serial.println();
#else
#define LOGD(...)
#define LOGE(...)
#endif
