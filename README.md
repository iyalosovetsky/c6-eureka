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


Pinouts
  
  output:
   - pwm Fan  ->  5
   - fan door conrtol -> 4

  input:
    - fan speed sensor -> 7

   


| Part name                            |                                                         | Further info              | 
|-------------------------------------|-------------------------------------------------------------------|---------------------------|
|ESP32-C6                    |      [![Raspberry Pi5                      ](https://arduino.ua/images/AOC975_pinout.jpg)  | [Link](https://arduino.ua/prod7813-plata-rozrobnika-esp32-c6-mini-4mb-type-c-wifi-6-bluetooth-5) |
