/**
* SAMD21G18A-I2C-Motor-Driver-Slave.ino
* Main Arduino sketch for the SAMD21G18A-I2C-Slave device project.
*
* @license MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include <Wire.h>

// Pin assignments for motor control and charger status.
const int leftMotorA = 15;        // Pin for left motor A.
const int leftMotorB = 16;        // Pin for left motor B.
const int rightMotorA = 17;       // Pin for right motor A.
const int rightMotorB = 18;       // Pin for right motor B.
const int motorDriverEnable = 4;  // Pin to enable/disable motor driver.
const int chargerStatusPinA = 5;  // Charger status pin A (stat1).
const int chargerStatusPinB = 6;  // Charger status pin B (stat2).

// Variables to store motor PWM values.
volatile int16_t motorA = 0;  // PWM value for motor A.
volatile int16_t motorB = 0;  // PWM value for motor B.

// Charger state constants.
const uint8_t NOT_CONNECTED = 1;      // Charger not connected.
const uint8_t CHARGING = 2;           // Charger is charging.
const uint8_t CHARGING_COMPLETE = 3;  // Charger has completed charging.

volatile uint8_t chargerState = NOT_CONNECTED;  // Variable to hold the current charger state.
volatile uint8_t lastRequestType = 0;           // Stores the last request type received from the master.

/**
* This function initializes the I2C interface and sets the pin modes for
* motor control and charger status.
*/
void setup() {
  Wire.begin(0x68);              // Initialize I2C as slave with address 0x68.
  Wire.onReceive(receiveEvent);  // Register event handler for receiving data.
  Wire.onRequest(requestEvent);  // Register event handler for sending data.

  // Pin mode definitions.
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(motorDriverEnable, OUTPUT);
  pinMode(chargerStatusPinA, INPUT);
  pinMode(chargerStatusPinB, INPUT);
}

/**
* Main program loop
* 
* The loop function checks charger status pins and updates the charger state.
* It also calls functions to control the motor speeds.
*/
void loop() {
  if (digitalRead(chargerStatusPinA) == HIGH && digitalRead(chargerStatusPinB) == HIGH) {
    chargerState = NOT_CONNECTED;
  } else if (digitalRead(chargerStatusPinA) == LOW && digitalRead(chargerStatusPinB) == HIGH) {
    chargerState = CHARGING;
  } else if (digitalRead(chargerStatusPinA) == HIGH && digitalRead(chargerStatusPinB) == LOW) {
    chargerState = CHARGING_COMPLETE;
  }

  controlMotorA(motorA);
  controlMotorB(motorB);
}

/**
* Event handler for receiving data over I2C.
* 
* This function handles incoming data from the master device and updates
* motor control settings or other parameters based on the received command.
* 
* @param numBytes The number of bytes received from the master.
*/
void receiveEvent(int numBytes) {
  if (numBytes < 1) return;  // Ensure there's at least 1 byte to read.

  uint8_t command = Wire.read();  // Read the command byte.
  lastRequestType = command;

  switch (command) {
    case 0x02:  // Command to enable the motor driver.
      digitalWrite(motorDriverEnable, HIGH);
      break;
    case 0x03:  // Command to disable the motor driver.
      digitalWrite(motorDriverEnable, LOW);
      break;
    case 0x04:  // Command to set motor A PWM value.
      if (numBytes >= 3) {
        motorA = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorA.
      }
      break;
    case 0x05:  // Command to set motor B PWM value.
      if (numBytes >= 3) {
        motorB = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorB.
      }
      break;
    case 0x08:  // Command to set both motor A and motor B PWM values.
      if (numBytes >= 5) {
        motorA = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorA.
        motorB = (Wire.read() << 8) | Wire.read();  // Combine high and low bytes for motorB.
      }
      break;
    default:
      break;  // Unknown command, do nothing.
  }
}

/**
* Event handler for sending data over I2C.
* 
* This function sends the requested data back to the master based on the last.
* received request type.
*/
void requestEvent() {
  switch (lastRequestType) {     // This variable is set in the receiveEvent.
    case 0x01:                   // Request for the charger state.
      Wire.write(chargerState);  // Send the charger state.
      break;
    case 0x06:                             // Request for motor A PWM value.
      Wire.write((uint8_t)(motorA >> 8));  // Send high byte.
      Wire.write((uint8_t)motorA);         // Send low byte.
      break;
    case 0x07:                             // Request for motor B PWM value.
      Wire.write((uint8_t)(motorB >> 8));  // Send high byte.
      Wire.write((uint8_t)motorB);         // Send low byte.
      break;
    case 0x09:  // Request for motor driver enabled state.
      Wire.write(digitalRead(motorDriverEnable) == HIGH ? 1 : 2);
      break;
    default:
      Wire.write(0);  // Send 0 for unknown requests.
      break;
  }
}

/**
* Controls the motor A based on the given PWM value.
* 
* This function sets the motor speed and direction based on the input value.
* A positive value moves the motor forward, negative moves it in reverse,
* and 0 stops the motor.
* 
* @param value The PWM value to control the motor (positive for forward, negative for reverse).
*/
void controlMotorA(int value) {
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
* Controls the motor B based on the given PWM value.
* 
* This function sets the motor speed and direction based on the input value.
* A positive value moves the motor forward, negative moves it in reverse,
* and 0 stops the motor.
* 
* @param value The PWM value to control the motor (positive for forward, negative for reverse).
*/
void controlMotorB(int value) {
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