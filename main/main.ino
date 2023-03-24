/*
* Authors: Jake Armstrong, Garrett Hart
* Date: 3/23/2023
*/
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>
#include <FastPID.h>
#include <FRAM_RINGBUFFER.h>
#include <FRAM.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "BluetoothSerial.h"
#include "HUSKYLENS.h"

/******** MOTORS **********/
#define STEERING_MAX 170
#define STEERING_MIN 10
#define STEERING_PIN 32
Servo steering;

#define SPEED_MIN 50
#define SPEED_MAX 120
#define SPEED_PIN 33
Servo motor;

/******** CAMERA **********/
#define HUSKY_SDA 21
#define HUSKY_SCL 22
#define HUSKY_ADR 0x32

/******** BLEUTOOTH **********/
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled.
#endif
#define USE_PIN //optional

String* device_name = new String("ECE362CarTeam07");
BluetoothSerial SerialBT;
const char *pin = "5188";


/******** CURRENT SENSOR **********/
Adafruit_INA219 ina219;


/********* BEGIN HELPER FUNCTIONS **********/
void startup_motor() {
  motor.attach(SPEED_PIN, 800, 2000);
  delay(1000);
  motor.write(0);
  delay(4000);
}

void startup_steering() {
  steering.attach(STEERING_PIN);
  steering.write(90);
}

void init_bluetooth() {
  SerialBT.begin(*device_name);
  Serial.printf("##########\nDevice Name: %s\n########", device_name->c_str());
  
  #ifdef USE_PIN // Optional 
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

}

void setSpeed(int speed) {
    if(speed < 0 || speed > 100) return;
    int val = map(speed, 0, 100, SPEED_MIN, SPEED_MAX);
    motor.write(val);
}

/********* END HELPER FUNCTIONS *********/



/******** ARDUINO SETUP FUNCTION **********/
void setup()
{
  Serial.begin(115200);
  
  init_bluetooth();
  startup_motor();
  startup_steering();  

  Serial.begin(115200);

  if (! ina219.begin()) {
    SerialBT.printf("Failed to find INA219 chip\n");
  }

  
  SerialBT.printf("Startup Complete\n");
}

/**
 * TODO: Handle Input from the HUSKYLens
 * TODO: Drive the car
 * TODO: Setup power recording
*/

/******** ARDUINO LOOP FUNCTION **********/
void loop()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");

  // steering.write(STEERING_MIN);
  // delay(2000);
  // steering.write(STEERING_MAX);
  delay(200);
	
}
