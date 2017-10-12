#define PIN_ECHO A1
#define PIN_TRIGGER A0
#define PIN_BUZZER 3
#define PIN_ENABLE_BUTTON 4
#define PIN_LEFT_MOTOR_FORWARD 5
#define PIN_LEFT_MOTOR_BACKWARD 9
#define PIN_LEFT_MOTOR_ENABLE 7
#define PIN_RIGHT_MOTOR_FORWARD 6
#define PIN_RIGHT_MOTOR_ENABLE 8
#define PIN_RIGHT_MOTOR_BACKWARD 10
#define PIN_BT_RX 12
#define PIN_BT_TX 11

#define ULTRASONIC_MEASURMENTS_ARRAY_SIZE 10
#define maxDistance 25
#define minDistance 20
#define DRIVE_VALUE_STEP 10

#include "SoftwareSerial.h"

SoftwareSerial bluetooth(PIN_BT_TX, PIN_BT_RX);

//#define SERIAL_DATA_EXCHANGE

int frontDistance = 0;
byte ultrasonicArrayPointer = 0;
float ultrasonicMeasurmentsSum = 0;
byte robotMode = 0;
byte previousRobotMode = 1;
int leftWheelSpeed = 100;
int rightWheelSpeed = 100;
enum {
  modeIdle = 0,
  modeKeepWallDistance,
  modeBluetooth,
  modeAvoidance
} driveMode;

void setup()
{
  //Initialize motor drive for output mode
  pinMode(PIN_LEFT_MOTOR_FORWARD, OUTPUT); 
  pinMode(PIN_LEFT_MOTOR_BACKWARD, OUTPUT); 
  pinMode(PIN_RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR_BACKWARD, OUTPUT);

  // initialize enable button
  pinMode(PIN_ENABLE_BUTTON, INPUT);
  digitalWrite(PIN_ENABLE_BUTTON, HIGH);

  // initialize buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);

  // initialize ultrasonic sensor
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
  
  // initialize bluetooth
  bluetooth.begin(115200);
  bluetooth.print("$$$");
  delay(100);
  bluetooth.println("U,9600,N");
  bluetooth.begin(9600);

#ifdef SERIAL_DATA_EXCHANGE
  Serial.begin(9600);
#endif
  
}

/*
 * Function to drive.
 */
void Drive(int speedL = 100, int speedR = 100, int time = 0)
{
#ifdef SERIAL_DATA_EXCHANGE
  Serial.print("Left motor speed: ");
  Serial.print(speedL);
  Serial.print(";Right motor speed: ");
  Serial.println(speedR);
#endif
  if (speedR > 0) {
    digitalWrite(PIN_RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(PIN_RIGHT_MOTOR_BACKWARD, LOW);
    analogWrite(PIN_RIGHT_MOTOR_FORWARD, speedR);
    analogWrite(PIN_RIGHT_MOTOR_BACKWARD, 0);
  }
  else if (speedR < 0) {
    digitalWrite(PIN_RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_RIGHT_MOTOR_BACKWARD, HIGH);
    analogWrite(PIN_RIGHT_MOTOR_FORWARD, 0);
    analogWrite(PIN_RIGHT_MOTOR_BACKWARD, -1 * speedR);
  }
  else {
    digitalWrite(PIN_RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_RIGHT_MOTOR_BACKWARD, LOW);
    analogWrite(PIN_RIGHT_MOTOR_FORWARD, 0);
    analogWrite(PIN_RIGHT_MOTOR_BACKWARD, 0);
  }
  if (speedL > 0) {
    digitalWrite(PIN_LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(PIN_LEFT_MOTOR_BACKWARD, LOW);
    analogWrite(PIN_LEFT_MOTOR_FORWARD, speedL);
    analogWrite(PIN_LEFT_MOTOR_BACKWARD, 0);
  }
  else if (speedL < 0) {
    digitalWrite(PIN_LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_LEFT_MOTOR_BACKWARD, HIGH);
    analogWrite(PIN_LEFT_MOTOR_FORWARD, 0);
    analogWrite(PIN_LEFT_MOTOR_BACKWARD, -1 * speedL);
  }
  else {
    digitalWrite(PIN_LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_LEFT_MOTOR_BACKWARD, LOW);
    analogWrite(PIN_LEFT_MOTOR_FORWARD, 0);
    analogWrite(PIN_LEFT_MOTOR_BACKWARD, 0);
  }
  delay(time);
}

/*
 * Function to drive foward
 */
void DriveForward(int speed = 100, int time = 10) {
  Drive(speed, speed, time);
}

/*
 * Stop the robot.
 */
void DriveStop()
{
  Drive(0, 0);
}

/*
 * Function to drive backwards
 */
void DriveBackward(int speed = 100, int time = 0)
{
  Drive(-1 * speed, -1 * speed, time);
}

/*
 * Measure front distance using ultrasonic sensor and return the distance in cm.
 */
int GetUSonicDistance()
{
  static float arrDistanceValues[ULTRASONIC_MEASURMENTS_ARRAY_SIZE];
  
  digitalWrite(PIN_TRIGGER, LOW);    // set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(PIN_TRIGGER, HIGH);  // set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);    // set trig port low level
  float distance = pulseIn(PIN_ECHO, HIGH);  // Read echo port high level time(unit:μs)
  /*
   * Distance(m) =(time(s) * 344(m/s)) / 2
   ****** The speed of sound is 344m/s.*******
   * ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
   * ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
   */
  distance = distance/58;
  
  ultrasonicMeasurmentsSum += distance - arrDistanceValues[ultrasonicArrayPointer];
  arrDistanceValues[ultrasonicArrayPointer] = distance;
  ultrasonicArrayPointer = (ultrasonicArrayPointer + 1) % ULTRASONIC_MEASURMENTS_ARRAY_SIZE;

  distance = ultrasonicMeasurmentsSum / ULTRASONIC_MEASURMENTS_ARRAY_SIZE;

#ifdef SERIAL_DATA_EXCHANGE
  // Output distance to console
  Serial.print("Distance: ");
  Serial.println(distance);
#endif
  
  int result = distance;
  return result;
}  

bool GetControlSignal() {
  static unsigned long buttonPressedTime;
  static bool buttonPressed = false;
  if (bluetooth.available()) {
    char inputChar = (char)bluetooth.read();
#ifdef SERIAL_DATA_EXCHANGE
    Serial.print(inputChar);
#endif
    // change mode to idle
    if (inputChar == ' ') {
      if (robotMode != modeIdle) {
        previousRobotMode = robotMode;
        robotMode = modeIdle;
      }
      else {
        robotMode = previousRobotMode;
      }
      return true;
    }
    // change mode to keep distance from the wall
    else if (inputChar == '1') {
      if (robotMode == modeIdle) {
        previousRobotMode = modeKeepWallDistance;
      }
      else {
        robotMode = modeKeepWallDistance;
      }
      return true;
    }
    // change mode to bluetooth remote
    else if (inputChar == '2') {
      if (robotMode == modeIdle) {
        previousRobotMode = modeBluetooth;
      }
      else {
        robotMode = modeBluetooth;
      }
      return true;
    }
    // change mode to free ride avoiding obstacles
    else if (inputChar == '3') {
      if (robotMode == modeIdle) {
        previousRobotMode = modeAvoidance;
      }
      else {
        robotMode = modeAvoidance;
      }
      return true;
    }
    // if in bluetooth mode get commands
    else if (robotMode == modeBluetooth or (robotMode == modeIdle and previousRobotMode == modeBluetooth)) {
      // increase speed
      if (inputChar == 'w') {
        if (leftWheelSpeed + DRIVE_VALUE_STEP < 256 and rightWheelSpeed + DRIVE_VALUE_STEP < 256) {
           leftWheelSpeed += DRIVE_VALUE_STEP;
           rightWheelSpeed += DRIVE_VALUE_STEP;
           return true;
        }
      }
      // decrease speed
      else if (inputChar == 's') {
        if (leftWheelSpeed - DRIVE_VALUE_STEP > -256 and rightWheelSpeed - DRIVE_VALUE_STEP > -256) {
           leftWheelSpeed -= DRIVE_VALUE_STEP;
           rightWheelSpeed -= DRIVE_VALUE_STEP;
           return true;
        }
      }
      // turn left
      else if (inputChar == 'a') {
        if (leftWheelSpeed <= rightWheelSpeed) {
          if (leftWheelSpeed - DRIVE_VALUE_STEP > -256) {
            leftWheelSpeed -= DRIVE_VALUE_STEP;
            return true;
          }
        }
        else {
          rightWheelSpeed += DRIVE_VALUE_STEP;
          return true;
        }
      }
      // turn right
      else if (inputChar == 'd') {
        if (rightWheelSpeed <= leftWheelSpeed) {
          if (rightWheelSpeed - DRIVE_VALUE_STEP > -256) {
            rightWheelSpeed -= DRIVE_VALUE_STEP;
            return true;
          }
        }
        else {
          leftWheelSpeed += DRIVE_VALUE_STEP;
          return true;
        }
      }
      // go straight
      else if (inputChar == 'r') {
        if (rightWheelSpeed <= 0 and leftWheelSpeed <= 0) {
          rightWheelSpeed = min(rightWheelSpeed, leftWheelSpeed);
          leftWheelSpeed = rightWheelSpeed;
        }
        else {
          rightWheelSpeed = max(rightWheelSpeed, leftWheelSpeed);
          leftWheelSpeed = rightWheelSpeed;
        }
        return true;
      }
    }
  }
  if (millis() - buttonPressedTime > 10) {
    if (!digitalRead(PIN_ENABLE_BUTTON)) {
      if (!buttonPressed) {
        buttonPressed = true;
        if (robotMode != modeIdle) {
          previousRobotMode = robotMode;
          robotMode = modeIdle;
        }
        else {
          robotMode = previousRobotMode;
        }
        buttonPressedTime = millis();
        digitalWrite(PIN_BUZZER, LOW);
        return true;
      }
    }
    else {
      if (buttonPressed) {
        buttonPressed = false;
        buttonPressedTime = millis();
        digitalWrite(PIN_BUZZER, HIGH);
      }
    }
  }
  return false;
}

void loop()
{
  if (GetControlSignal() or robotMode == modeKeepWallDistance) {
    if (robotMode == modeKeepWallDistance) {
      frontDistance = GetUSonicDistance();
      if (frontDistance > maxDistance) {
        DriveForward();
      }
      else if (frontDistance < minDistance) {
        DriveBackward();
      }
      else {
        DriveStop();
      }
#ifdef SERIAL_DATA_EXCHANGE
      Serial.println(frontDistance);
#endif
    }
    else if (robotMode == modeBluetooth) {
      Drive(leftWheelSpeed, rightWheelSpeed);
    }
    else if (robotMode == modeIdle) {
      DriveStop();
    }
  }
}

