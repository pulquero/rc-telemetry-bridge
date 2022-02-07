set ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli update --additional-urls https://dl.espressif.com/dl/package_esp32_index.json
arduino-cli core install esp32:esp32 --additional-urls https://dl.espressif.com/dl/package_esp32_index.json
arduino-cli lib install --git-url https://github.com/256dpi/arduino-mqtt
arduino-cli lib install --git-url https://github.com/h2zero/NimBLE-Arduino
arduino-cli lib install --git-url https://github.com/me-no-dev/AsyncTCP
arduino-cli lib install --git-url https://github.com/me-no-dev/ESPAsyncWebServer
