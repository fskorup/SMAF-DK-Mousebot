#include <Wire.h>

const int leftMotorA = 15;
const int leftMotorB = 16;
const int rightMotorA = 17;
const int rightMotorB = 18;
const int motorDriverEnable = 4;
const int chargerStatusPinA = 5;  // stat1
const int chargerStatusPinB = 6;  // stat2

// Variables to store motor PWM values
volatile int16_t motorA = 0;
volatile int16_t motorB = 0;

// Charger state constants
const uint8_t NOT_CONNECTED = 1;
const uint8_t CHARGING = 2;
const uint8_t CHARGING_COMPLETE = 3;

// Variable to hold the current charger state
volatile uint8_t chargerState = NOT_CONNECTED;

void setup() {
  Wire.begin(0x68);              // Initialize I2C as slave with address 0x68
  Wire.onReceive(receiveEvent);  // Register event handler for receiving data
  Wire.onRequest(requestEvent);  // Register event handler for sending data

  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(motorDriverEnable, OUTPUT);
  pinMode(chargerStatusPinA, INPUT);
  pinMode(chargerStatusPinB, INPUT);
}

void loop() {
  // Simulate changing charger states for demonstration purposes (this logic can be replaced with actual hardware readings)
  // static unsigned long lastUpdateTime = 0;
  // if (millis() - lastUpdateTime >= 2400) {
  //   lastUpdateTime = millis();
  //   chargerState = chargerState % 3 + 1;  // Cycle through 1 (NOT_CONNECTED), 2 (CHARGING), 3 (CHARGING_COMPLETE)
  // }

  if (digitalRead(chargerStatusPinA) == HIGH && digitalRead(chargerStatusPinB) == HIGH) {
    chargerState = 1;  // Not connected.
  } else if (digitalRead(chargerStatusPinA) == LOW && digitalRead(chargerStatusPinB) == HIGH) {
    chargerState = 2;  // Charging.
  } else if (digitalRead(chargerStatusPinA) == HIGH && digitalRead(chargerStatusPinB) == LOW) {
    chargerState = 3;  // Charging complete.
  }

  controlRightMotor(motorA);
  controlLeftMotor(motorB);
}

volatile uint8_t lastRequestType = 0;

// Event handler function for receiving data from the master
void receiveEvent(int numBytes) {
  if (numBytes < 1) return;  // Ensure there's at least 1 byte to read

  uint8_t command = Wire.read();  // Read the command byte
  lastRequestType = command;

  switch (command) {
    case 0x02:  // Command to enable the motor driver
      digitalWrite(motorDriverEnable, HIGH);
      break;
    case 0x03:  // Command to disable the motor driver
      digitalWrite(motorDriverEnable, LOW);
      break;
    case 0x04:  // Command to set motor A PWM value
      if (numBytes >= 3) {
        motorA = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorA
      }
      break;
    case 0x05:  // Command to set motor B PWM value
      if (numBytes >= 3) {
        motorB = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorB
      }
      break;
    case 0x08:  // Command to set both motor A and motor B PWM values
      if (numBytes >= 5) {
        motorA = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorA
        motorB = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorB
      }
      break;
    default:
      // Unknown command, do nothing
      break;
  }
}

// Event handler function for sending data to the master
void requestEvent() {
  // Directly handle the data sending based on an earlier request
  switch (lastRequestType) {     // This variable should be set in the receiveEvent
    case 0x01:                   // Request for the charger state
      Wire.write(chargerState);  // Send the charger state
      break;
    case 0x06:                             // Request for motor A PWM value
      Wire.write((uint8_t)(motorA >> 8));  // Send high byte
      Wire.write((uint8_t)motorA);         // Send low byte
      break;
    case 0x07:                             // Request for motor B PWM value
      Wire.write((uint8_t)(motorB >> 8));  // Send high byte
      Wire.write((uint8_t)motorB);         // Send low byte
      break;
    case 0x09:  // Request for motor driver enabled state
      Wire.write(digitalRead(motorDriverEnable) == HIGH ? 1 : 2);
      break;
    default:
      Wire.write(0);  // Send 0 for unknown requests
      break;
  }
}

/**
* Controls the right motor based on the given PWM value.
* This function sets the motor speed and direction based on the input value.
* 
* @param value The PWM value to control the motor (positive for forward, negative for reverse).
*/
void controlRightMotor(int value) {
  int speed = map(abs(value), 0, 255, 0, 255);
  if (value > 0) {
    analogWrite(rightMotorA, speed);
    analogWrite(rightMotorB, 0);
  } else if (value < 0) {
    analogWrite(rightMotorA, 0);
    analogWrite(rightMotorB, speed);
  } else {
    analogWrite(rightMotorA, 0);
    analogWrite(rightMotorB, 0);
  }
}

/**
* Controls the left motor based on the given PWM value.
* This function sets the motor speed and direction based on the input value.
* 
* @param value The PWM value to control the motor (positive for forward, negative for reverse).
*/
void controlLeftMotor(int value) {
  int speed = map(abs(value), 0, 255, 0, 255);
  if (value > 0) {
    analogWrite(leftMotorA, speed);
    analogWrite(leftMotorB, 0);
  } else if (value < 0) {
    analogWrite(leftMotorA, 0);
    analogWrite(leftMotorB, speed);
  } else {
    analogWrite(leftMotorA, 0);
    analogWrite(leftMotorB, 0);
  }
}