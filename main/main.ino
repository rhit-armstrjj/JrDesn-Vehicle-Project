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
#include <vector>

/**
 * Control Loop
*/
uint32_t loopDelay = 20;

/******** MOTORS **********/
#define STEERING_MAX 170
#define STEERING_MIN 10
#define STEERING_PIN 32
Servo steering;
float Ksp = 0.1, Ksi = 0.5, Ksd = 0, Hz = 1000/loopDelay;
int steeringOutputBits = 8;
bool steeringOutputSigned = true;
FastPID steeringPID(Ksp, Ksi, Ksd, Hzs, steeringOutputBits, steeringOutputSigned);

#define SPEED_MIN 50
#define SPEED_MAX 120
#define SPEED_PIN 33
Servo motor;
float Kvp = 0.1, Kvi = 0.5, Kvd = 0;
int speedOutputBits = 7;
bool speedOutputSigned = false;
FastPID speedPID(Kvp, Kvi, Kvd, Hzv, speedOutputBits, speedOutputSigned);

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
std::vector<String> errorLog;


/******** CURRENT SENSOR **********/
Adafruit_INA219 ina219;

/******** Huskylens **************/
HUSKYLENS camera;


//TODO: Setup FRAM for logging power.
/*******/

/********* BEGIN HELPER FUNCTIONS **********/
/**
 * Sets up motor contorl.
*/
void startup_motor() {
  motor.attach(SPEED_PIN, 800, 2000);
  delay(1000);
  motor.write(0);
  delay(4000);
}

/**
 * Sets up Steering servo.
*/
void startup_steering() {
  steering.attach(STEERING_PIN);
  steering.write(90);

}

/**
 * Starts up Huskylens
*/
void startup_huskylens() {
  Wire.begin(HUSKY_ADR);
  while(!camera.begin(Wire)) {
    SerialBT.println("Warning: Huskylens not connected. Retrying.");
    delay(400);
  }
}

/**
 * Initializes bluetooth
*/
void init_bluetooth() {
  SerialBT.begin(*device_name);
  SerialBT.printf("##########\nDevice Name: %s\n########", device_name->c_str());
  delay(100);
  
  #ifdef USE_PIN // Optional 
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

}

/**
 * Set speed by percentage.
*/
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

/**
 * Writes telemetry to screen. Needs to write status messages
*/
int transmitStatusInfo() {
  struct PowerInfo info;
  getPowerInfo(&info);

  size_t length = SerialBT.printf(
    "##################\nTime: %d ms\r\nBus Voltage:   %f V\r\nShunt Voltage: %f mV\r\nLoad Voltage:  %f V\r\nCurrent:       %f mA\r\nPower: %f mW\r\n##########\r\n",
    millis(), info.busVoltage, info.shuntVoltage, info.loadVoltage, info.current, info.power
  );
  for(int i = 0; i < errorLog; i++) {
    SerialBT.println(errorLog.at(i));
  }
  return length;
}

/**
 * For handling Huskylens issues.
*/
bool getHuskyArrowX(int* x) {
  HUSKYLENSResult arrow = camera.read();
  if(arrow.command != COMMAND_RETURN_ARROW) return false;
}

/**
 * For mapping arrow target to steering.
 * Output should be between STEERING_MIN & STEERING_MAX
*/
int getSteeringSetpoint(int steeringMapped) {
  // TODO Determine how steering should be given a setpoint.
  return -1;
} 

/**
 * For mapping the speed based on the arrowX
 * Output should be between SPEED_MIN & SPEED_MAX
*/
int getSpeedSetpoint(int arrowX) {
  // TODO Determine how steering should be given a setpoint.
  
  return -1;
} 

/********* END HELPER FUNCTIONS *********/



/******** ARDUINO SETUP FUNCTION **********/
void setup()
{
  Serial.begin(115200);
  
  init_bluetooth();
  startup_motor();
  startup_steering(); 
  startup_huskylens(); 

  Serial.begin(115200);

  if (! ina219.begin()) {
    SerialBT.printf("Failed to find INA219 chip\r\n");
  }

  SerialBT.println("################");
  SerialBT.println("Startup Complete");
  SerialBT.println("################");
  delay(900);
  SerialBT.write('\033[2J');
  SerialBT.write('\033[1;1H');
  delay(100);
}

int loops = 0;
/******** ARDUINO LOOP FUNCTION **********/
void loop()
{
  // Telemetry Transmission
  int powerSize = transmitStatusInfo();

  int arrowX;
  // End loop if camera doesn't have a result ready.
  if(!camera.available() && !getHuskyArrowX(&arrowX)) {
    delay(loopDelay);
    SerialBT.write('\033[2J');
    SerialBT.write('\033[1;1H');
    return;
  }
  
  // TODO Need help with control theory.

  // Map steering to center the arrow to the top of the screen
  int steeringMapped = map(arrowX-160, -160, 160, STEERING_MIN, STEERING_MAX);
  uint8_t steeringAngle = steeringPID.step(getSteeringSetpoint(steeringMapped), steeringMapped);
  steering.write(steeringAngle);

  // feedback is mapped to arrow bc speed should determined by angle of steering column
  int speedMapped = map(arrowX-160, -160, 160, SPEED_MIN, SPEED_MAX);
  int targetSpeed = speedPID.step(getSpeedSetpoint(speedMapped), speedMapped);
  motor.write(targetSpeed);


  // Clear Terminal
  delay(loopDelay);
  SerialBT.write('\033[2J');
  SerialBT.write('\033[1;1H');
	loops++;
}
