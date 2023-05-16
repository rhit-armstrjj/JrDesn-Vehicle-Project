
/*
* Authors: Jake Armstrong, Garrett Hart
* Date: 3/23/2023
*/
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <FastPID.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "BluetoothSerial.h"
#include "HUSKYLENS.h"
#include <vector>
#include <MsgPack.h>
#include <MsgPacketizer.h>
#include <LinkedList.h>

/**
 * Control Loop
*/
uint32_t loopDelay = 60;
unsigned int computeTime = 0;

float Ksp = 0.452, Ksi = 0.0003, Ksd = 0.00, Hz = 1000/loopDelay;

/******** MOTORS **********/
#define STEERING_MAX 170
#define STEERING_MIN 10
#define STEERING_PIN 32

Servo steering;
int steeringOutputBits = 8;
bool steeringOutputSigned = true;
FastPID steeringPID;
int mapped_steering = 0;

#define SPEED_MIN 49
#define SPEED_MAX 120
#define SPEED_PIN 33
Servo motor;
int driveSpeed = 0;


/******** BLEUTOOTH **********/
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled.
#endif
#define USE_PIN //optional

MsgPack::Unpacker unpacker; // For Unpacking & Packing Telemetry
MsgPack::Packer packer;


#define REQUEST_TELEMETRY 4
const int VEHICLE_STATS = 12;

struct Telemetry {
  unsigned int ms;
  unsigned int raceTime;

  bool arrowExists;
  int arrowHead;
  int arrowTail;

  unsigned int speed;
  unsigned int servoValue;

  float voltage;
  float current;
  float power;
  float energy;

  float P;
  float I;
  float D;

  MSGPACK_DEFINE(ms, raceTime, arrowExists, arrowHead, arrowTail, speed, servoValue, voltage, current, power, energy, P,I,D);
};

String *device_name = new String("ECE362CarTeam07");
BluetoothSerial SerialBT;
const char *pin = "5188";
std::vector<String> errorLog;

boolean arrowLost = false;
int arrowTarget = 0;
int arrowEnd = 0;

const uint8_t recv_index = 0x12;


/******** CURRENT SENSOR **********/
Adafruit_INA219 ina219;

/**
 * Power Info
 * Shunt Voltage: mV (voltage across current sensor) (Omitted)
 * busVoltage: V (total voltage)
 * current: mA
 * loadVoltage: V (Voltage across on low side of current sensor resistor) (Omitted)
 * power: mW
*/
struct PowerInfo {
  float busVoltage;
  float current;
  float power;
  MSGPACK_DEFINE(busVoltage, current, power);
};

/** DO NOT CHANGE THE FOLLOWING VARIABLES WITHOUT LOCKING MUTEX*/
PowerInfo currentPower;
float currentAverage;
float energyExpended;
/**End mutex-protected variables.*/

// Protects shared variables between cores.
SemaphoreHandle_t powerMutex = NULL;

//Queue of currents.
LinkedList<float> currentList;

/******** Huskylens **************/
#define HUSKY_SDA 21
#define HUSKY_SCL 22
#define HUSKY_ADR 0x32
HUSKYLENS camera;


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

/********* BEGIN HELPER FUNCTIONS **********/
/**
 * Sets up motor contorl.
*/
void startup_motor() {
  motor.attach(SPEED_PIN, 800, 2000);
  delay(1000);
  motor.write(0);
  delay(5000);
}

/**
 * Sets up Steering servo.
*/
void startup_steering() {
  // Load Ksp, Ksi, & Ksd

  steeringPID = FastPID(Ksp, Ksi, Ksd, Hz, steeringOutputBits, steeringOutputSigned);
  steering.attach(STEERING_PIN);
  steering.write(90);

}

/**
 * Sets the steering angle from -100 to 100
*/
void setSteering(int angle) {
  mapped_steering = map(angle, -100, 100, STEERING_MIN, STEERING_MAX );
  steering.write(mapped_steering);
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
    driveSpeed = val;
    motor.write(val);
}

/**
 * Handles getting power info.
*/
void getPowerInfo(struct PowerInfo *data) { 

  float busvoltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  float power = ina219.getPower_mW();

  if(currentList.size() == (1000 / loopDelay)) { // Keep list average at around one second
    currentList.pop();
  }
  
  currentList.add(0, current);

  float sum = 0;
  for(int i = 0; i < currentList.size(); i++) {
    sum += currentList.get(i);
  }

  if(xSemaphoreTake(powerMutex, portTICK_PERIOD_MS * 10)) { // Take mutex, timeout 10ms
    *data = {
      busvoltage,
      current,
      power
    };

    currentAverage = (sum / currentList.size());
    energyExpended = energyExpended + (power*1000/loopDelay);

    xSemaphoreGive(powerMutex); // Give mutex back
  }

}

/**
 * For handling requests to Huskylens and updates arrow state.
 * Stops car if arrow is lost.
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

  arrowTarget = arrow.xTarget;
  arrowEnd = arrow.xOrigin;

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

  if(arrowLost) setSpeed(0); // Failsafe if the car falls off of the track.
  
  driveTime = millis() - startDriveTime;
  if(driveTime >= 90000) state = Waiting; 
}

/**
 * Idle state for car. Motors are actively off.
*/
void waitingStateLoop() {
  setSpeed(0);
  setSteering(0);
  getPowerInfo(&currentPower);
  driveTime = 0;
  arrowLost = false;
}
/********* END HELPER FUNCTIONS *********/

/******** ARDUINO SETUP FUNCTION **********/
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Wire Began");
  init_bluetooth();
  Serial.println("Bluetooth Init");
  startup_motor();
  Serial.println("Motor Init");
  startup_steering(); 
  Serial.println("Steering Init");
  startup_huskylens();
  Serial.println("Camera Init");
  powerMutex = xSemaphoreCreateMutex();

  state = Waiting;

  Serial.println("Ready!");

  // Handles when a packet is received over bluetooth.
  MsgPacketizer::subscribe(SerialBT, REQUEST_TELEMETRY,
      [](int payload_id, MsgPack::map_t<String,String> payload)
      {
        Serial.println("Packet Received.");
        // send received data back in one-line
        if(xSemaphoreTake(powerMutex, portTICK_PERIOD_MS * 10)) {

          Telemetry stuff = {
            millis(),
            0,// driveTime,

            0,// arrowLost,
            0,// arrowTarget,
            0,// arrowEnd,

            0,// driveSpeed,
            0,// mapped_steering,

            currentPower.busVoltage,
            currentPower.current,
            energyExpended,

            0,// Ksp,
            0,// Ksi,
            0// Ksd
          };

          MsgPacketizer::send(Serial, VEHICLE_STATS, payloadId, stuff);
        }
      }
  );

  // // Loop 2
  // xTaskCreatePinnedToCore(
  //   (TaskFunction_t) loop,
  //   "TELEMETRY",
  //   1000,
  //   NULL,
  //   tskIDLE_PRIORITY,
  //   NULL,
  //   0
  // );
}

/******** ARDUINO LOOP FUNCTION **********/
void loop()
{
  unsigned int loopTime = millis();
  getPowerInfo(&currentPower);

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

  MsgPacketizer::parse();
  
  computeTime = loopTime-millis();
  
  delay(loopDelay - computeTime);
}

uint8_t* telemetryBuffer;
size_t telemetryBufferSize = 0;

void telemetryLoop() {
  MsgPacketizer::parse();
}


