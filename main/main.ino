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

float Ksp = 0.435, Ksi = 0.0001, Ksd = 0.00, Hz = 1000/loopDelay;

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


const uint8_t REQUEST_TELEMETRY = 0x34;
const uint8_t VEHICLE_STATS = 0x20;
const uint8_t START_RACE = 2;
const uint8_t STOP_RACE = 3;
const uint8_t UPDATE_PID = 69;

struct Telemetry {
  MsgPack::str_t ms, raceTime, arrowExists, arrowHead, arrowTail, 
  speed, servoValue, voltage, current, power, energy, P,I,D, state;
  unsigned int k_ms;
  unsigned int k_raceTime;

  bool k_arrowExists;
  int k_arrowHead;
  int k_arrowTail;

  unsigned int k_speed;
  unsigned int k_servoValue;

  float k_voltage;
  float k_current;
  float k_power;
  float k_energy;

  float k_P;
  float k_I;
  float k_D;

  int k_state;

  MSGPACK_DEFINE_MAP(ms, k_ms, raceTime, k_raceTime, arrowExists, k_arrowExists,
   arrowHead, k_arrowHead, arrowTail, k_arrowTail, speed, k_speed, servoValue, k_servoValue,
   voltage, k_voltage, current, k_current, power, k_power, energy, k_energy, P, k_P, I, k_I, D, k_D, state,k_state);
};

String *device_name = new String("ECE362CarTeam07");
BluetoothSerial SerialBT;
const char *pin = "5188";

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
};

/** DO NOT CHANGE THE FOLLOWING VARIABLES WITHOUT LOCKING MUTEX*/
PowerInfo currentPower;
float currentAverage = 0;
float energyExpended = 0;
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
    Serial.println("# Warning: Huskylens not connected. Retrying.");
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
      driveSpeed = 0;
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
    energyExpended = energyExpended + (power/1000.0)*(loopDelay/1000.0);

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
    setSpeed(8);
  } else {
    setSpeed(4);
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
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  state = Waiting;

  Serial.println("Ready!");

  // Handles when a telemetry request packet is received over bluetooth.
  MsgPacketizer::subscribe(SerialBT, REQUEST_TELEMETRY,
    [&](const MsgPack::map_t<String, int> unused) {
      // Serial.println("Packet Received.");
      // send received data back in one-line

      Telemetry stuff = {
          "ms",
          "raceTime",
          "arrowExists",
          "arrowTarget",
          "arrowEnd",
          "driveSpeed",
          "servoSetting",
          "busVoltage",
          "current",
          "power",
          "energySpent",
          "P","I","D","state",
          millis(),
          driveTime,// driveTime,

          arrowLost,// arrowLost,
          arrowTarget,// arrowTarget,
          arrowEnd,// arrowEnd,

          driveSpeed,// driveSpeed,
          mapped_steering,// mapped_steering,

          currentPower.busVoltage,
          ina219.getCurrent_mA(),// Current currant
          currentPower.power,
          energyExpended,

          Ksp,// Ksp,
          Ksi,// Ksi,
          Ksd,// Ksd

          state
        };

        MsgPacketizer::send(SerialBT, VEHICLE_STATS, stuff);
        
      }
  );

  MsgPacketizer::subscribe(SerialBT, START_RACE, 
    [&](const MsgPack::map_t<String, int> unused) {
      Serial.println("Starting Race");
      // TODO: Set state to start race state.
      state = State::StartRace;
    }
  );

  MsgPacketizer::subscribe(SerialBT, STOP_RACE, 
    [&](const MsgPack::map_t<String, int> unused) {
      Serial.println("Stopping Race");
      state = State::Waiting;
    }
  );
  
  //Subscribe to Update PID Data
  MsgPacketizer::subscribe(SerialBT, UPDATE_PID, 
    [](MsgPack::map_t<String, float> PID) {
      Serial.println("Updating PID");
      Ksp = PID["Ksp"];
      Ksi = PID["Ksi"];
      Ksd = PID["Ksd"];
      Serial.print("Ksp ");
      Serial.println(PID["Ksp"]);
      steeringPID = FastPID(Ksp, Ksi, Ksd, Hz, steeringOutputBits, steeringOutputSigned);
    }
  );

}

/******** ARDUINO LOOP FUNCTION **********/
void loop()
{
  unsigned int loopTime = millis();
  getPowerInfo(&currentPower);

  // if(SerialBT.available() > 0) Serial.printf("%d Bytes Available\r\n", SerialBT.available());

  MsgPacketizer::parse();
  

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

  computeTime = loopTime-millis();
  
  delay(loopDelay - computeTime);
}
