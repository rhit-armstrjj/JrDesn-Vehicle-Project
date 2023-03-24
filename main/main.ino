/*
* Authors: Jake Armstrong
* Date: 3/23/2023
*/
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>

#include <FastPID.h>

#include <FRAM_RINGBUFFER.h>
#include <FRAM.h>

#include <Adafruit_INA219.h>
#include "BluetoothSerial.h"

#define STEERING_MAX 170
#define STEERING_MIN 10
#define STEERING_PIN 32
Servo steering;

#define SPEED_MIN 50
#define SPEED_MAX 120
#define SPEED_PIN 33
Servo motor;

#include "HUSKYLENS.h"
#define HUSKY_SDA 21
#define HUSKY_SCL 22
#define HUSKY_ADR 0x32

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

void setup()
{
  Serial.begin(115200);
  
  init_bluetooth();
  startup_motor();
  startup_steering();

  SerialBT.printf("Startup Complete");
  
}

/**
 * TODO: Handle Input from the HUSKYLens
 * TODO: Drive the car
*/
void loop()
{
  steering.write(STEERING_MIN);
  delay(5000);
  steering.write(STEERING_MAX);
  delay(5000);
	
}
