// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee Dimmable light bulb.
 *
 * The example demonstrates how to use Zigbee library to create an end device with
 * dimmable light end point.
 * The light bulb is a Zigbee end device, which is controlled by a Zigbee coordinator.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by [FaBjE](https://github.com/FaBjE) based on examples by [Jan Procházka](https://github.com/P-R-O-C-H-Y/)
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include "driver/pulse_cnt.h"

/* Zigbee OTA configuration */
#define OTA_UPGRADE_RUNNING_FILE_VERSION    0x01010100  // Increment this value when the running image is updated
#define OTA_UPGRADE_DOWNLOADED_FILE_VERSION 0x01010101  // Increment this value when the downloaded image is updated
#define OTA_UPGRADE_HW_VERSION              0x0101      // The hardware version, this can be used to differentiate between different hardware versions

/* Zigbee dimmable light configuration */
#define ZIGBEE_LIGHT_ENDPOINT 11
#define ZIGBEE_LIGHT2_ENDPOINT 15
#define ZIGBEE_DIMMABLELIGHT_ENDPOINT 10
#define TEMP_SENSOR_ENDPOINT_NUMBER 13
// #define WIND_SPEED_SENSOR_ENDPOINT_NUMBER 13
#define ANALOG_DEVICE_ENDPOINT_NUMBER 12
#define ANALOG_DEVICE2_ENDPOINT_NUMBER  14
#define ZIGBEE_ILLUMINANCE_SENSOR_ENDPOINT 9

 

#define GLITCH_NS_DEFAULT 1000

uint8_t led = RGB_BUILTIN;
uint8_t button = BOOT_PIN;
//uint8_t pin_a = BOOT_PIN; // for counter

// uint8_t pwmFan = 23;
uint8_t pwmFan = 5;
// uint8_t pinSpeedFan = 3;
uint8_t pinSpeedFan = BOOT_PIN;
uint8_t pinCover = 4;
uint8_t analogPin = A0;


// ESP32 PWM dimming settings
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 12; 
// variables for dimming
uint32_t maxLevelTransformed = pow(2, PWM_RESOLUTION)-1; // internal max brightness level
uint32_t levelTransformed;                               // zigbee brightness value that got transformed into internal brightness level
uint32_t lastLevelTransformed;                           // internal brightness level of last brightness change
int flowDirection = 0;                           // internal brightness level of last brightness change
int lastflowDirection =2;
int fanOnOff = 0;                           // internalon on off
int lastFanOnOff =2;




pcnt_unit_handle_t unit= NULL;
pcnt_channel_handle_t chan_a= NULL;
int fan_pulse_counter = 0;
int oldcount = 22;
// int fanCounterVal = 10;
uint16_t fanCounterVal = 10;
static uint32_t fanCounterTimer = millis();


int16_t low_limit = INT16_MIN;
int16_t high_limit = INT16_MAX;    
//uint16_t PWM_DIVIDER_MAX = 59800/2;
//uint16_t PWM_DIVIDER_MAX = (maxLevelTransformed+1)*0.45;
uint16_t PWM_DIVIDER_MAX = (maxLevelTransformed+1)/2;



uint16_t glitch_time = GLITCH_NS_DEFAULT;    

ZigbeeDimmableLight zbDimmableLight = ZigbeeDimmableLight(ZIGBEE_DIMMABLELIGHT_ENDPOINT);
ZigbeeLight zbFanDirSwitch = ZigbeeLight(ZIGBEE_LIGHT_ENDPOINT); //ig added
ZigbeeLight zbFanOnOffSwitch = ZigbeeLight(ZIGBEE_LIGHT2_ENDPOINT); //ig added

// ZigbeeWindSpeedSensor zbWindSpeedSensor = ZigbeeWindSpeedSensor(WIND_SPEED_SENSOR_ENDPOINT_NUMBER); //ig added
// ZigbeeAnalog zbAnalogDevice = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);//ig added
ZigbeeAnalog zbAnalogFan = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER );
ZigbeeAnalog zbAnalogCtrlFan = ZigbeeAnalog(ANALOG_DEVICE2_ENDPOINT_NUMBER );
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);//ig added
ZigbeeIlluminanceSensor zbIlluminanceSensor = ZigbeeIlluminanceSensor(ZIGBEE_ILLUMINANCE_SENSOR_ENDPOINT);



// /************************ WindSpeed sensor *****************************/
// static void windspeed_sensor_value_update(void *arg) {
//   for (;;) {
//     // Read wind speed sensor value (simulated now by temperature sensor)
//     float windspeed = temperatureRead()+4.7;
//     log_v("Wind speed sensor value: %.2fm/s", windspeed);
//     log_v("Wind speed FAKE  sensor value: %.2fm/s", oldcount*2);
    
//     // Update windspeed value in Windspeed sensor EP
//      Serial.printf("wind_sensor_value_update: Updated counter sensor value to %.2f %.2f  on perMin \r\n", windspeed, (float) oldcount*2);
//     // zbWindSpeedSensor.setWindSpeed(windspeed); -- todo
//     zbWindSpeedSensor.setWindSpeed((oldcount*2)%5000);
//     delay(1000);
//   }
// }


/********************* Arduino functions **************************/

int updatePcntFanRPM(){
  pcnt_unit_get_count(unit, &fan_pulse_counter);
  if ((millis() - fanCounterTimer) > 10000) {
    fanCounterTimer =millis();
  //  zbAnalogFan.reportAnalogInput
     ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  }
  // todo remove below
  oldcount++;
  fan_pulse_counter+=oldcount;
  return fan_pulse_counter;
}


/************************ FanRpm sensor *****************************/
static void fan_speed_sensor_value_update(void *arg) {
  for (;;) {
    updatePcntFanRPM();
    zbAnalogFan.setAnalogInput(fan_pulse_counter);
    //
    // Update windspeed value in Windspeed sensor EP
    // Serial.printf("[fan_speed_sensor_value_update]: Updated counter sensor value to %.2f %.2f   \r\n", fan_pulse_counter, (float) oldcount);
    delay(1000);
  }
}

/********************* Illuminance sensor **************************/
static void illuminance_sensor_value_update(void *arg) {
  for (;;) {
    // read the raw analog value from the sensor
    // int lsens_analog_raw = analogRead(illuminance_sensor_pin);
    // Serial.printf("[Illuminance Sensor] raw analog value: %d\r\n", lsens_analog_raw);

    // conversion into zigbee raw illuminance value (typically between 0 in darkness and 50000 in direct sunlight)
    // depends on the value range of the raw analog sensor values and will need calibration for correct lux values
    // for demonstration purpose map the 12-bit ADC value (0-4095) to Zigbee illuminance range (0-50000)
    // int lsens_illuminance_raw = map(lsens_analog_raw, 0, 4095, 0, 50000);
    // Serial.printf("[Illuminance Sensor] raw illuminance value: %d\r\n", lsens_illuminance_raw);

    // according to zigbee documentation the formular 10^(lsens_illuminance_raw/10000)-1 can be used to calculate lux value from raw illuminance value
    // Note: Zigbee2MQTT seems to be using the formular 10^(lsens_illuminance_raw/10000) instead (without -1)
    // int lsens_illuminance_lux = round(pow(10, (lsens_illuminance_raw / 10000.0)) - 1);
    // int lsens_illuminance_raw = map(fanCounterVal, 0, 4095, 0, 50000); //ig added
    // Serial.printf("[Illuminance Sensor] fanCounterVal value: %d\r\n", fanCounterVal*10);
    // int lsens_illuminance_lux = round(pow(10, (lsens_illuminance_raw / 10000.0)) - 1);
    // Serial.printf("[Illuminance Sensor] lux value: %d lux\r\n", lsens_illuminance_lux);

    // Update illuminance in illuminance sensor EP
    zbIlluminanceSensor.setIlluminance(fanCounterVal*10);  // use raw illuminance here!

    delay(1000);  // reduce delay (in ms), if you want your device to react more quickly to changes in illuminance
  }
}



static void temp_sensor_value_update(void *arg) {
  for (;;) {
    // Read temperature sensor value
    float tsens_value = temperatureRead();
    // Serial.printf("[temp_sensor_value_update] Updated temp sensor value to %.2f°C\r\n", (float) (abs(fanCounterVal*3)%40));
    // Update temperature value in Temperature sensor EP
    zbTempSensor.setTemperature(tsens_value);
    delay(1000);
  }
}

void onAnalogOutputChange(float analog_output) {
  Serial.printf("Received analog output change: %.1f\r\n", analog_output);
  if (analog_output<7 && zbFanOnOffSwitch.getLightState()) {
    zbFanOnOffSwitch.setLight(false);  
    zbFanOnOffSwitch.restoreLight();
  } else  if (analog_output>10 && !zbFanOnOffSwitch.getLightState()) {
    zbFanOnOffSwitch.setLight(true);  
    zbFanOnOffSwitch.restoreLight();
  }

  uint8_t level=zbFanOnOffSwitch.getLightState()? (analog_output>255?255:analog_output):0;
  setLight(zbFanOnOffSwitch.getLightState(),   level);
}

/********************* RGB LED functions **************************/
void setFanDirection(bool value) {//ig added
  lastflowDirection=flowDirection;
  if (value) {
    Serial.println(" setLED 1");
    flowDirection=1;
  }
  else {
    Serial.println(" setLED 0");
    flowDirection=0;
  }
  setLight(fanOnOff,  zbDimmableLight.getLightLevel());
}


/********************* RGB LED functions **************************/
void setFanOnOff(bool value) {//ig added
  lastFanOnOff=fanOnOff;
  if (value) {
    Serial.println(" setLED 1");
    fanOnOff=1;
  }
  else {
    Serial.println(" setLED 0");
    fanOnOff=0;
  }
  setLight(fanOnOff,  zbDimmableLight.getLightLevel());
}




uint16_t transformBrightness(int value) {
    //float gamma = 2.2;
    //return (int) (pow(value / 254.0, gamma) * maxLevelTransformed);

    int dif= PWM_DIVIDER_MAX * (1-(value>=255?254:(value<=0?1:value))/255.0);
    uint16_t val;

    if (flowDirection) {
        val=PWM_DIVIDER_MAX+dif;
    } else {
        val=PWM_DIVIDER_MAX-dif;
    }
    
    

    
    return val  ;


}






void setLight(bool state, uint8_t level) {
  levelTransformed = transformBrightness(level);

  // ignore brightness changes between two equal values
  if ((levelTransformed == lastLevelTransformed ) &&(flowDirection == lastflowDirection )) return;  
  if (!state) {
    digitalWrite(pinCover, LOW); // Turn the LED off
    rgbLedWrite(led, 10, 0, 0);
    // ledcFade(pwmFan, levelTransformed, 0, 100); // smooth zigbee brightness changes happen every 100 ms
    ledcWrite(pwmFan, 0); // smooth zigbee brightness changes happen every 100 ms
    return;
  }
  if (flowDirection)
     rgbLedWrite(led, 0, 0, level);
  else   
    rgbLedWrite(led, 0, level, 0);
  // ledcFade(pwmFan, lastLevelTransformed, levelTransformed, 100); // smooth zigbee brightness changes happen every 100 ms
  if (level>0) {
    digitalWrite(pinCover, HIGH); // Turn the LED on
  }
  ledcWrite(pwmFan, levelTransformed); // smooth zigbee brightness changes happen every 100 ms

 // log brightness changes
  Serial.print("[Brightness] Zigbee: ");
  Serial.print(level);
  // Serial.print(" | Internal before: ");
  // Serial.print(lastLevelTransformed);
  Serial.print(" | pwm target: ");
  Serial.print(levelTransformed);
  Serial.print(" / ");
  Serial.println(maxLevelTransformed);

  // remember internal level for next brightness change for smooth zigbee brightness changes
  lastLevelTransformed = levelTransformed;  
}
 



// Create a task on identify call to handle the identify function
void identify(uint16_t time) {
  static uint8_t blink = 1;
  log_d("Identify called for %d seconds", time);
  if (time == 0) {
    // If identify time is 0, stop blinking and restore light as it was used for identify
    zbDimmableLight.restoreLight();
    return;
  }
  rgbLedWrite(led, 255 * blink, 255 * blink, 255 * blink);
  blink = !blink;
}

// Create a task on identify call to handle the identify function
void identify2(uint16_t time) {
  static uint8_t blink = 1;
  log_d("Identify2 called for %d seconds", time);
  if (time == 0) {
    // If identify time is 0, stop blinking and restore light as it was used for identify
    zbDimmableLight.restoreLight();
    return;
  }
  rgbLedWrite(led, 255 * blink, 255 * blink, 255 * blink);
  blink = !blink;
}




static void pcnt_example_config_unit(pcnt_unit_config_t *config, int unit_num) {
   config->low_limit = -5000;
   config->high_limit = 5000;
  config->flags.accum_count = true;  //Enable accumulating the count
  config->intr_priority = 3;
}

void initPCNT(){
  // Unit config
  // pcnt_unit_config_t unit_config = {
  //     .low_limit = low_limit,
  //     .high_limit = high_limit,
  // };
  // unit_config.flags.accum_count = true;

  pcnt_unit_config_t unit_config;
  pcnt_example_config_unit(&unit_config, 0);

  // Create unit
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));
  

  // Set watch points at low and high limits to auto-accumulate overflows.
  // pcnt_unit_add_watch_point(unit, low_limit);
  // pcnt_unit_add_watch_point(unit, high_limit);

  // Glitch filter setup
  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = glitch_time,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

  // Channel A setup
  pcnt_chan_config_t chan_a_config = {
      // .edge_gpio_num = pinSpeedFan,
      .edge_gpio_num = pinSpeedFan,
      .level_gpio_num = -1,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_config, &chan_a));



  // Set edge and level actions for both channels
  // ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  // ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

  // Enable, clear and start the PCNT unit.
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));


}



void epTempSetup()
{
  zbTempSensor.setManufacturerAndModel("ihorYalosovetskyi", "tempSensor"); //ig added
  // // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
  zbTempSensor.setMinMaxValue(-10, 80); //ig added
  //   // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setTolerance(0.2);
  // // Optional: Time cluster configuration (default params, as this device will revieve time from coordinator)
  // zbTempSensor.addTimeCluster();
  Zigbee.addEndpoint(&zbTempSensor);
  // Serial.println("Added Zigbee temp sensor endpoint to Zigbee Core");
  delay(100);
  
}

void epDimmableSetup()
{
  zbDimmableLight.setManufacturerAndModel("ihorYalosovetskyi", "ZBDimmable"); // Optional: Set Zigbee device name and model
  // zbDimmableLight.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION, OTA_UPGRADE_DOWNLOADED_FILE_VERSION, OTA_UPGRADE_HW_VERSION); // Add OTA client to the light bulb
  zbDimmableLight.onLightChange(setLight); // Set callback function for light change on hass
  zbDimmableLight.onIdentify(identify);// Optional: Set callback function for device identify
  Zigbee.addEndpoint(&zbDimmableLight);
  delay(100);
}

void epFanDirSetup()
{
  zbFanDirSwitch.setManufacturerAndModel("ihorYalosovetskyi", "ZBFanDir"); //Optional: set Zigbee device name and model
  zbFanDirSwitch.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION, OTA_UPGRADE_DOWNLOADED_FILE_VERSION, OTA_UPGRADE_HW_VERSION);// Add OTA client to the light bulb
  zbFanDirSwitch.onLightChange(setFanDirection); // Optional: // Set callback function for light change  on hass
  zbFanDirSwitch.onIdentify(identify2); 
  Zigbee.addEndpoint(&zbFanDirSwitch); //Add endpoint to Zigbee Core
  delay(100);
}  

void epFanOnOffSetup()
{
  zbFanOnOffSwitch.setManufacturerAndModel("ihorYalosovetskyi", "ZBFanOnOff"); //Optional: set Zigbee device name and model
  zbFanOnOffSwitch.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION, OTA_UPGRADE_DOWNLOADED_FILE_VERSION, OTA_UPGRADE_HW_VERSION);// Add OTA client to the light bulb
  zbFanOnOffSwitch.onLightChange(setFanOnOff); // Optional: // Set callback function for light change  on hass
  zbFanOnOffSwitch.onIdentify(identify); 
  Zigbee.addEndpoint(&zbFanOnOffSwitch); //Add endpoint to Zigbee Core
  delay(100);
}  

 void epFanSetup() {
  zbAnalogFan.setManufacturerAndModel("ihorYalosovetskyi", "C6Fan"); //ig added
  zbAnalogFan.addAnalogInput();// Set up analog input //ig added
  
  
  // zbAnalogFan.setAnalogInputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFan.setAnalogInputApplication(ESP_ZB_ZCL_AI_APP_TYPE_RPM);
  zbAnalogFan.setAnalogInputDescription("RPM");
  zbAnalogFan.setAnalogInputResolution(1);  
  zbAnalogFan.setAnalogInputMinMax(-999999, 999999);

  


  // zbAnalogFan.setAnalogInputReporting(10, 60, 10);

  Zigbee.addEndpoint(&zbAnalogFan); // Add endpoint to Zigbee Core

  delay(100);
 }



void eFanCtrlSetup() {
  zbAnalogCtrlFan.setManufacturerAndModel("ihorYalosovetskyi", "C6CtrlFan"); //ig added
  // Set up analog output
  zbAnalogCtrlFan.addAnalogOutput();
  zbAnalogCtrlFan.setAnalogOutputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogCtrlFan.setAnalogOutputDescription("Fan Speed (RPM)");
  zbAnalogCtrlFan.setAnalogOutputResolution(1);

  // Set the min and max values for the analog output which is used by HA to limit the range of the analog output
  zbAnalogCtrlFan.setAnalogOutputMinMax(-10000, 10000);  //-10000 to 10000 RPM

  // If analog output cluster is added, set callback function for analog output change
  zbAnalogCtrlFan.onAnalogOutputChange(onAnalogOutputChange);


  // zbAnalogFan.setAnalogInputReporting(10, 60, 10);

  Zigbee.addEndpoint(&zbAnalogCtrlFan); // Add endpoint to Zigbee Core

  delay(100);
 }


 void epIlluminanceSetup() {
  zbIlluminanceSensor.setManufacturerAndModel("ihorYalosovetskyi", "Illuminance");// Optional: Set Zigbee device name and model
  zbIlluminanceSensor.setPowerSource(ZB_POWER_SOURCE_MAINS); // Optional: Set power source (choose between ZB_POWER_SOURCE_MAINS and ZB_POWER_SOURCE_BATTERY), defaults to unknown
  zbIlluminanceSensor.setMinMaxValue(0, 50000);// Set minimum and maximum for raw illuminance value (0 min and 50000 max equals to 0 lux - 100,000 lux)
  zbIlluminanceSensor.setTolerance(1);// Optional: Set tolerance for raw illuminance value
  
  // zbIlluminanceSensor.setReporting(0, 60, 0);
  Zigbee.addEndpoint(&zbIlluminanceSensor);// Add endpoint to Zigbee Core
  delay(100);
 }



void setup() {
  Serial.begin(115200);
  pinMode(pinCover, OUTPUT);  // Set pin 13 as an output
  digitalWrite(pinCover, LOW); // Turn the LED on
  ledcAttach(pwmFan, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(pwmFan, 0);

  // Init RMT and leave light OFF
  rgbLedWrite(led, 0, 0, 0);

  // Init button for factory reset
  pinMode(button, INPUT_PULLUP);

  initPCNT();

  epFanDirSetup();
  epFanOnOffSetup();
  epDimmableSetup();
  epIlluminanceSetup();
  epTempSetup();
  epFanSetup();
  eFanCtrlSetup();
  
  
  






  // Optional: set Zigbee device name and model

    // Add analog clusters to Zigbee Analog according your needs
  // zbAnalogDevice.addAnalogInput();
  // zbAnalogDevice.addAnalogOutput();

  


    // Optional: set Zigbee device name and model
  // zbWindSpeedSensor.setManufacturerAndModel("Espressif", "ZigbeeWindSensor"); //ig added
    //  Set minimum and maximum windspeed measurement value in m/s
  // zbWindSpeedSensor.setMinMaxValue(0, 50000); //ig added
  // Set tolerance for windspeed measurement in m/s (lowest possible value is 0.01 m/s)
  // zbWindSpeedSensor.setTolerance(1);//ig added











  //  Add endpoint to Zigbee Core
  //  Zigbee.addEndpoint(&zbWindSpeedSensor);//ig added

    // Add endpoints to Zigbee Core
  // Zigbee.addEndpoint(&zbAnalogDevice); //ig added

    // Add endpoint to Zigbee Core
  // Zigbee.addEndpoint(&zbTempSensor);//ig added

//Open network for 180 seconds after boot
  Zigbee.setRebootOpenNetwork(180); //ig added https://wiki.seeedstudio.com/xiao_esp32c6_zigbee_arduino/

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("Tasks:");
  xTaskCreate(fan_speed_sensor_value_update, "fan_speed_sensor_value_update", 2048, NULL, 10, NULL);
  xTaskCreate(illuminance_sensor_value_update, "illuminance_sensor_value_update", 2048, NULL, 10, NULL);
  xTaskCreate(temp_sensor_value_update, "temp_sensor_value_update", 2048, NULL, 10, NULL);


    // Start Wind speed sensor reading task
  // xTaskCreate(windspeed_sensor_value_update, "wind_speed_sensor_update", 2048, NULL, 10, NULL);

    // Start Temperature sensor reading task
  // xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 2048, NULL, 10, NULL);
  // Start illuminance sensor reading task
  // xTaskCreate(illuminance_sensor_value_update, "illuminance_sensor_update", 2048, NULL, 10, NULL);

  // Set reporting interval for windspeed measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (WindSpeed change in m/s)
  // if min = 1 and max = 0, reporting is sent only when windspeed changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or windspeed changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of windspeed change
  //min_interval,max_interval,  delta
  //  zbWindSpeedSensor.setReporting(0, 60, 0);

    // Start Zigbee OTA client query, first request is within a minute and the next requests are sent every hour automatically

    // zbIlluminanceSensor.setReporting(1, 0, 1000);
     

    
     
  zbFanDirSwitch.requestOTAUpdate();
}

void loop() {
  static uint32_t timeCounter = 0;
  if (fanCounterTimer> millis()) fanCounterTimer=millis();


     if ((millis() - fanCounterTimer) > 10000) //6 sec
    {
      fanCounterTimer=millis();
      fanCounterVal=fan_pulse_counter;
      Serial.printf("[Illuminance Sensor] fanCounterVal value: %d\r\n", fanCounterVal*10);
      zbIlluminanceSensor.setIlluminance(fanCounterVal*10);  // use raw illuminance here!
      zbIlluminanceSensor.report();
      zbAnalogFan.reportAnalogInput();
      zbTempSensor.report();
     }
    

  if (digitalRead(button) == LOW) {  // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    
    
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 3000) {
        // If key pressed for more than 3secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        Zigbee.factoryReset();
      }
    }
    zbDimmableLight.setLightLevel(zbDimmableLight.getLightLevel() + 50);

  }
  
  delay(100);
}

// import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

// export default {
//     zigbeeModel: ['C6Fan','tempSensor','Illuminance','ZBDimmable','ZBLightBulb'],
//     model: 'C6Fan',
//     vendor: 'ihorYalosovetskyi',
//     description: 'ESP32-C6 controller',
//     meta: { multiEndpoint: true },
//     extend: [
//         m.deviceEndpoints({
//             "endpoints":{
//                 "fan_pwm":10,
//                 "fan_dir":11,
//                 "fan_speed":12,
//                 "internal_temp":13,
//                 "system_init":99}
//         }), 
//         m.onOff({
//             "powerOnBehavior":false,
//             "endpointNames":["fan_pwm"],
//             "description": "Fan on/off",
//         }), 
//         m.onOff({
//             "powerOnBehavior":false,
//             "endpointNames":["fan_dir"],
//             "description": "Fan direction",
//         }), 
//         m.light({
//              endpointNames: ["fan_pwm"], // Назвемо сутність з димером 'dimmable'
//              powerOnBehavior: false,
//              "description": "Fan speed"
//          }),        
//         m.temperature({
//             "endpointNames": ["internal_temp"],
//             "description": "Internal Temp",
//         }),
//         m.binary({
//             "name":"system_init",
//             "cluster":"genBinaryInput",
//             "attribute":"presentValue",
//             "reporting":{"attribute":"presentValue","min":"MIN","max":"MAX","change":1},
//             "valueOn":["ON",1],
//             "valueOff":["OFF",0],
//             "description":"System Initialised",
//             "access":"STATE_GET",
//             "endpointName":"system_init"
//         }),
//         m.numeric({
//             "name": "fan_speed",
//             "unit": "rpm",
//             "description": "fan rpm",
//             "cluster": "genAnalogInput",
//             "attribute": "presentValue",
//             "reporting": {attribute: "presentValue", min: 5, max: 360000, change: 1},
//             "endpointName": "fan_speed",
//         }),        
//     ],    
// };