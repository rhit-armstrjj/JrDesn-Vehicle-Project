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
  SerialBT.printf("##########\nDevice Name: %s\n########", device_name->c_str());
  delay(100);
  
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

/**
 * Power Info
 * Shunt Voltage: mV
 * busVoltage: V
 * current: mA
 * loadVoltage: V
 * power: mW
*/
struct PowerInfo {
  float shuntVoltage;
  float busVoltage;
  float current;
  float loadVoltage;
  float power;
};


/**
 * Handles getting power info.
*/
void getPowerInfo(struct PowerInfo *data) { 

  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  *data = {
    shuntvoltage,
    busvoltage,
    ina219.getCurrent_mA(), 
    busvoltage + (shuntvoltage / 1000),
    ina219.getPower_mW()
  };
}


int transmitPowerInfo() {
  struct PowerInfo info;
  getPowerInfo(&info);

  size_t length = SerialBT.printf(
    "##################\nTime: %d ms\r\nBus Voltage:   %f V\r\nShunt Voltage: %f mV\r\nLoad Voltage:  %f V\r\nCurrent:       %f mA\r\nPower: %f mW\r\n",
    millis(), info.busVoltage, info.shuntVoltage, info.loadVoltage, info.current, info.power
  );

  delay(200);
  return length;
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
    SerialBT.printf("Failed to find INA219 chip\r\n");
  }

  SerialBT.println("################");
  SerialBT.println("Startup Complete");
  SerialBT.println("################");
}

/**
 * TODO: Handle Input from the HUSKYLens
*/

int loops = 0;
/******** ARDUINO LOOP FUNCTION **********/
void loop()
{

  
  int powerSize = transmitPowerInfo();

  steering.write(STEERING_MIN);
  delay(5000);
  steering.write(STEERING_MAX);
  delay(5000);
  
  for(int i = 0; i < powerSize; i++) {
    SerialBT.write('\b');
  }
  delay(10);
	loops++;
}
