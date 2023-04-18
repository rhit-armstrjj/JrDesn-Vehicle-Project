

/*
* Authors: Jake Armstrong, Garrett Hart
* Date: 3/23/2023
*/
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <FastPID.h>
#include <FRAM_RINGBUFFER.h>
#include <FRAM.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "BluetoothSerial.h"
#include "HUSKYLENS.h"
#include <vector>
#include <MsgPacketizer.h>
#include <MsgPack.h>
#include <Adafruit_EEPROM_I2C.h>
#include <Adafruit_FRAM_I2C.h>

/**
 * Control Loop
*/
uint32_t loopDelay = 60;
float Ksp = 0.375, Ksi = 0.06, Ksd = 0.00, Hz = 1000/loopDelay;

/******** MOTORS **********/
#define STEERING_MAX 170
#define STEERING_MIN 10
#define STEERING_PIN 32
Servo steering;
int steeringOutputBits = 8;
bool steeringOutputSigned = true;
FastPID steeringPID;

#define SPEED_MIN 49
#define SPEED_MAX 120
#define SPEED_PIN 33
Servo motor;


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
boolean arrowLost = false;


/******** CURRENT SENSOR **********/
Adafruit_INA219 ina219;

/******** Huskylens **************/
HUSKYLENS camera;
#define HUSKY_SDA 21
#define HUSKY_SCL 22
#define HUSKY_ADR 0x32

/******** FRAM *********/
#define FRAM_ADR 0x50
Adafruit_EEPROM_I2C fram;

/**
 * Internal States
 * State Definitions:
 *  - waiting: Idle, not moving.
 *  - startRace: captures current time in millis and transitions to driving.
 *  - driving: captures current time.
 * 
*/
enum State {
  Waiting = 0,
  StartRace = 1,
  Driving = 2,
};

int state = 0; 
bool newState = false;
unsigned long driveTime = 0;
unsigned long startDriveTime = 0;

//TODO: Setup FRAM for saving states.
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
  // Load Ksp, Ksi, & Ksd from FRAM

  steeringPID = FastPID(Ksp, Ksi, Ksd, Hz, steeringOutputBits, steeringOutputSigned);
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
    SerialBT.println("# Warning: Huskylens not connected. Retrying.");
    delay(400);
  }

  camera.requestArrowsLearned();
}

/**
 * Initializes bluetooth
*/
void init_bluetooth() {
  SerialBT.begin(*device_name);
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
  MSGPACK_DEFINE(shuntVoltage, busVoltage, current, loadVoltage, power);
};
PowerInfo currentPower;

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

/********** FRAM HELPERS **********/
float readFloatFromFRAM(int idx) {
  float f; 
  uint8_t buffer[4];
  fram.read(idx*sizeof(float), buffer, sizeof(float));

  memcpy((void *) &f, buffer, 4);
  return f;
}

void writeFloatToFRAM(int idx, float f) {
  uint8_t buffer[4];
  memcpy(buffer, (void *) &f, 4);
  fram.write(idx*sizeof(float), buffer, sizeof(float));
}

void initFRAM() {

  fram.begin(FRAM_ADR);
  // Load default if no data in FRAM.
  if(readFloatFromFRAM(0) == 0) {
    writeFloatToFRAM(0, Ksp);
    writeFloatToFRAM(1, Ksi);
    writeFloatToFRAM(2, Ksd);
  } else {
    Ksp = readFloatFromFRAM(0);
    Ksi = readFloatFromFRAM(1);
    Ksd = readFloatFromFRAM(2);
  }
}

/**
 * For handling Huskylens issues.
*/
int getHuskyArrowX() {
  Serial.print("Request: ");
  bool res = camera.request(1);
  Serial.println(res);
  Serial.print("Available: ");
  Serial.println(camera.available());

  HUSKYLENSResult arrow = camera.read();
  
  if(arrow.command != COMMAND_RETURN_ARROW) {
    //Stop Car?
    arrowLost = true;
    return 160;
  }
  //printResult(arrow);
  return arrow.xTarget;
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
        SerialBT.println(String("#")+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        SerialBT.println(String("#")+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        SerialBT.println("# Object unknown!");
    }
}

/**
 * Starts the Race
*/
void startRaceLoop() {
  startDriveTime = millis();
  state = Driving;
}

/**
 * Driving State for Car
*/
void driveStateLoop() {
    //Arrow x is between 0 320
  int arrowX = getHuskyArrowX();

  // Map steering to center the arrow to the top of the screen
  int steeringMapped = arrowX-160;
  //SerialBT.print("Arrow X: ");
  //SerialBT.println(steeringMapped);
  int steerPD = steeringPID.step(0, steeringMapped);
  setSteering(steerPD);
  // feedback is mapped to arrow bc speed should determined by angle of steering column
  if(abs(steeringMapped) < 20) {
    setSpeed(10);
  } else {
    setSpeed(5);
  }
  if(arrowLost) setSpeed(0);
  driveTime = millis() - startDriveTime;
  if(driveTime >= 90000) state = Waiting; 
}

void waitingStateLoop() {
  setSpeed(0);
  setSteering(0);
  driveTime = 0;
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


  state = Waiting;
  MsgPacketizer::publish(SerialBT, 0x34, Ksp, Ksi, Ksd, state, driveTime, currentPower);
  MsgPacketizer::subscribe(SerialBT, 0x12, Ksp, Ksi, Ksd, state, driveTime, currentPower);

  // Loop 2
  xTaskCreatePinnedToCore(
    (TaskFunction_t) loop2,
    "TELEMETRY",
    500,
    NULL,
    tskIDLE_PRIORITY,
    NULL,
    0
  );
}

/******** ARDUINO LOOP FUNCTION **********/
void loop()
{
  // Telemetry Transmission
  //int powerSize = transmitStatusInfo();

  switch(state) {
    case StartRace:
      startRaceLoop();
    case Driving:
      driveStateLoop();
      break;

    case Waiting:
    default:
      state = Waiting;
      waitingStateLoop();
      break;
  }
  
  // Clear Terminal
  delay(loopDelay);
}

void telemetryLoop() {
  getPowerInfo(&currentPower);
  MsgPacketizer::update();
  
  //Update FRAM
  writeFloatToFRAM(0, Ksp);
  writeFloatToFRAM(1, Ksi);
  writeFloatToFRAM(2, Ksd);  

}

TaskFunction_t loop2() {
  while(1) {
    
    telemetryLoop();

  }

}


