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
uint32_t loopDelay = 70;

/******** MOTORS **********/
#define STEERING_MAX 170
#define STEERING_MIN 10
#define STEERING_PIN 32
Servo steering;
float Ksp = 0.72, Ksi = 0.0, Ksd = 0.03, Hz = 1000/loopDelay;
int steeringOutputBits = 8;
bool steeringOutputSigned = true;
FastPID steeringPID(Ksp, Ksi, Ksd, Hz, steeringOutputBits, steeringOutputSigned);

#define SPEED_MIN 49
#define SPEED_MAX 120
#define SPEED_PIN 33
Servo motor;
float Kvp = 0.65, Kvi = 0.04, Kvd = 0;
int speedOutputBits = 7;
bool speedOutputSigned = false;
FastPID speedPID(Kvp, Kvi, Kvd, Hz, speedOutputBits, speedOutputSigned);


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
#define HUSKY_SDA 21
#define HUSKY_SCL 22
#define HUSKY_ADR 0x32


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
 * Sets the steering angle from -100 to 100
*/
void setSteering(int angle) {
  steering.write(map(angle, -100, 100, STEERING_MIN, STEERING_MAX ));
}

/**
 * Starts up Huskylens
*/
void startup_huskylens() {
  while(!camera.begin(Wire)) {
    SerialBT.println("Warning: Huskylens not connected. Retrying.");
    delay(400);
  }

  camera.requestArrowsLearned();
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
    if(speed == 0) {
      motor.write(0);
      return;
    }
    int val = map(speed, 1, 100, SPEED_MIN, SPEED_MAX);
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
  for(int i = 0; i < errorLog.size(); i++) {
    length += SerialBT.println(errorLog.at(i));
  }
  return length;
}

/**
 * For handling Huskylens issues.
*/
int getHuskyArrowX() {
  SerialBT.print("Request: ");
  SerialBT.println(camera.request(1));
  SerialBT.print("Available: ");
  SerialBT.println(camera.available());
  
  HUSKYLENSResult arrow = camera.read();
  printResult(arrow);
  return arrow.xTarget;
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
  
  return abs(arrowX);
} 

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        SerialBT.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        SerialBT.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        SerialBT.println("Object unknown!");
    }
}
/********* END HELPER FUNCTIONS *********/



/******** ARDUINO SETUP FUNCTION **********/
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  init_bluetooth();
  startup_motor();
  startup_steering(); 
  startup_huskylens(); 

  if (!ina219.begin(&Wire)) {
    SerialBT.printf("Failed to find INA219 chip\r\n");
  }

  SerialBT.println("################");
  SerialBT.println("Startup Complete");
  SerialBT.println("################");
  delay(900);
}

int loops = 0;
/******** ARDUINO LOOP FUNCTION **********/
void loop()
{
  // Telemetry Transmission
  //int powerSize = transmitStatusInfo();

  //Arrow x is between 0 320
  int arrowX = getHuskyArrowX();

  // Map steering to center the arrow to the top of the screen
  int steeringMapped = arrowX-160;
  SerialBT.print("Arrow X: ");
  SerialBT.println(steeringMapped);
  int steerPD = steeringPID.step(0, steeringMapped);
  setSteering(steerPD);

  // feedback is mapped to arrow bc speed should determined by angle of steering column
  //int speedMapped = map(arrowX-160, -160, 160, SPEED_MIN, SPEED_MAX);
  //int targetSpeed = speedPID.step(getSpeedSetpoint(speedMapped), speedMapped);
  int speed = map(abs(steeringMapped), 0, 160, 10, 4);
  setSpeed(speed);


  // Clear Terminal
  delay(loopDelay);
	loops++;
}
