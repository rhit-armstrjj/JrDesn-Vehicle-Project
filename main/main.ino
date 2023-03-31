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
uint32_t loopDelay = 60;

/******** MOTORS **********/
#define STEERING_MAX 170
#define STEERING_MIN 10
#define STEERING_PIN 32
Servo steering;
float Ksp = 0.375, Ksi = 0.06, Ksd = 0.00, Hz = 1000/loopDelay;
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
boolean arrowLost = false;


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
    "%d, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f|\r\n",
    millis(), info.busVoltage, info.shuntVoltage, info.loadVoltage, info.current, info.power
  );
  // for(int i = 0; i < errorLog.size(); i++) {
  //   length += SerialBT.println(errorLog.at(i));
  // }
  return length;
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
    SerialBT.printf("# Failed to find INA219 chip\r\n");
  }

  SerialBT.println("####################");
  SerialBT.println("# Startup Complete #");
  SerialBT.println("####################");
  SerialBT.printf("#{\n\"device_name\": \"%s\",\n", device_name->c_str());
  SerialBT.printf("#  \"pid\": [%f,%f,%f]\n#}\n", Ksp, Ksi, Ksd);
  SerialBT.println("#{\"csv\": \"\n time (ms), bus_voltage (mV), shunt_voltage (V), load_voltage (V), total_current (mA), power (mW)\n");
  delay(3900);
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
  transmitStatusInfo();
  // Clear Terminal
  delay(loopDelay);
	loops++;
}
