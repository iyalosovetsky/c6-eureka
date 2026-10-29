# c6-eureka

zigbee espc6  bath fan controller

Open in arduino IDE ver 2.3.6

Set 
  - choose ESP32C6 dev Module
  - USB CDC on but -> enabled
  - erase all flash before scetch upload -> true
  - flash mode -> QIO
  - flash size -> 4MB
  - partition scheme -> "Zigbee 4MB with spiffs"
  - Zigbee mode -> Zigbee ED


Zigbee OTA update (no need to open the box)

  1. Bump `OTA_UPGRADE_RUNNING_FILE_VERSION` (and `..._DOWNLOADED_...`) in `main/main.ino`.
  2. Build with arduino-cli:

         arduino-cli compile --fqbn "esp32:esp32:esp32c6:ZigbeeMode=ed,PartitionScheme=zigbee" --output-dir build main

  3. Wrap it, same version as in step 1 (also writes `c6-index.json` next to it):

         python3 tools/make_ota.py build/main.ino.bin 0x01010400 build/c6-0x01010400.ota

  4. Copy the `.ota` file and `c6-index.json` to `<z2m data dir>/ota/` on the Home Assistant host,
     then run there `sudo sh tools/z2m_ota.sh check` and `sudo sh tools/z2m_ota.sh update`.
     The update takes about 80 minutes; c6 reboots at the end and the script turns the fan back on.


Pinouts
  
  output:
  
   - pwm Fan  ->  5
   - outdoor cover relay -> 4

  input:

    - fan speed sensor -> 7

   


| Part name                            |                                                         | Further info              | 
|-------------------------------------|-------------------------------------------------------------------|---------------------------|
|ESP32-C6                    |      [![Raspberry Pi5                      ](https://arduino.ua/images/AOC975_pinout.jpg)  | [Link](https://arduino.ua/prod7813-plata-rozrobnika-esp32-c6-mini-4mb-type-c-wifi-6-bluetooth-5) |
