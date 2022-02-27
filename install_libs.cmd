curl -o arduino-cli.zip -L https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip
powershell Expand-Archive -Force -Path arduino-cli.zip -DestinationPath .

set ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli update --additional-urls https://dl.espressif.com/dl/package_esp32_index.json
arduino-cli core install esp32:esp32 --additional-urls https://dl.espressif.com/dl/package_esp32_index.json
arduino-cli lib install --git-url https://github.com/256dpi/arduino-mqtt
arduino-cli lib install --git-url https://github.com/h2zero/NimBLE-Arduino
arduino-cli lib install --git-url https://github.com/me-no-dev/AsyncTCP
arduino-cli lib install --git-url https://github.com/me-no-dev/ESPAsyncWebServer

curl -o esp32fs.zip -L https://github.com/me-no-dev/arduino-esp32fs-plugin/releases/download/1.0/ESP32FS-1.0.zip
set ARDUINO_TOOLS=%HOMEPATH%\Documents\Arduino\tools
mkdir %ARDUINO_TOOLS%
powershell Expand-Archive -Force -Path esp32fs.zip -DestinationPath %ARDUINO_TOOLS%
