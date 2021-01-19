/*
   A PID based Line Follower BOT
   @author : Shashank Sahu

   Uses QTRSensor library for POLULU SENSOR Reading
   QTR-SENSOR : https://github.com/pololu/qtr-sensors-arduino

*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////      MOTOR CONNECTION SEMANTICS     ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
                                                        +-----------+
                                                        |           |
                                            +----------->   LEFT    +----+
  +--------------+                          |           |   MOTOR   |    |
  |              |                          |  +-------->           +----+
  |             5+--------+                 |  |        |           |
  |              |        |        +--------+--+----+   +-----------+
  |             6+-----+  |        |                |
  |              |     |  +-------->IN1             |
  |              |     |           |                |
  |     UNO      |     +----------->IN2     MOTOR   |
  |              |                 |        DRIVER  |
  |              |     +----------->IN3             |
  |              |     |           |                |
  |              |     |   +------->IN4             |
  |             9+-----+   |       |                |
  |              |         |       +--------+--+----+   +-----------+
  |            10+---------+                |  |        |           |
  |              |                          |  +-------->  RIGHT    +----+
  +--------------+                          |           |  MOTOR    |    |
                                            +----------->           +----+
                                                        |           |
                                                        +-----------+
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <QTRSensors.h>

#define IN1 5
#define IN2 6
#define IN3 9
#define IN4 10




//////////////////////////////////////////////////////
//////////////// CONSTANTS ///////////////////////////
//////////////////////////////////////////////////////
const uint8_t SENSOR_COUNT = 6;
const uint16_t IDEAL_POS = 2500;
const int CALIB_SPEED = 80;
const int BASE_SPEED = 130;
const int THRESHOLD = 200;
const int MAX_SPEED = 200;
///////////////// PID CONSTANTS /////////////////////
const float KP = 0.02;
const float KD = 0.35;
const float KI = 0.0;
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////



QTRSensors qtr;
int lastError = 0;
int LS;
int RS;
uint16_t sensorValues[SENSOR_COUNT];

void setup() {
  Serial.begin(9600);
  // Setting the pin MODES
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  for (int i = A0; i <= A5; i++) {
    pinMode(i, INPUT);
  }
  ///////////////////////////////////////////////////
  ////////////// Sensor Calibration /////////////////
  //////////////////////////////////////////////////
  delay(2000);
  manualCalibration();
  stopBot();
  delay(2000);
  ///////////////////////////////////////////////////
}
void loop() {
  specialCases();
  applyPID();
  moveForward();
}

void applyPID() {
  uint16_t position = qtr.readLineBlack(sensorValues); // Reading Line Position
  int error = position - IDEAL_POS; // Calculating Error
  float PError = error * KP;
  float DError = (error - lastError) * KD;
  float adjust = PError + DError;
  LS = BASE_SPEED - adjust;
  RS = BASE_SPEED + adjust;
  LS = constrain(LS, 0, MAX_SPEED);
  RS = constrain(RS, 0, MAX_SPEED);
  lastError = error;
}

boolean specialCases() {
  boolean flag = false;
  // 90 degree right turn
  if (sensorValues[0] > THRESHOLD) {
    rotateRight();
    flag = true;
  }

  // 90 degree left turn
  if (sensorValues[5] > THRESHOLD) {
    rotateLeft();
    flag = true;
  }
  // Extra Special Cases As per need
  if (sensorValues[0] < THRESHOLD && sensorValues[1] < THRESHOLD && sensorValues[2] < THRESHOLD && sensorValues[3] < THRESHOLD && sensorValues[4] < THRESHOLD && sensorValues[5] < THRESHOLD) {
    rotateLeft();
    flag = true;
  }
  if (sensorValues[0] > THRESHOLD && sensorValues[1] > THRESHOLD && sensorValues[2] > THRESHOLD && sensorValues[3] > THRESHOLD && sensorValues[4] > THRESHOLD && sensorValues[5] > THRESHOLD) {
    rotateRight();
    flag = true;
  }
  return flag;
}

void manualCalibration() {
  Serial.println("QTR configuration started:");
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5
  }, SENSOR_COUNT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  moveAround();
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}

void rotateLeft() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 255);
  analogWrite(IN3, 255);
  analogWrite(IN4, 0);
  delay(100);
}

void rotateRight() {
  analogWrite(IN1, 255);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 255);
  delay(100);
}


void rotate180() {
  analogWrite(IN1, 255);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 255);
  delay(200);
}

void moveForward() {
  analogWrite(IN1, LS);
  analogWrite(IN2, 0);
  analogWrite(IN3, RS);
  analogWrite(IN4, 0);
}

void moveAround() {
  analogWrite(IN1, CALIB_SPEED);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, CALIB_SPEED);
}

void stopBot() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}
