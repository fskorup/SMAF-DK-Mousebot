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

// Pin assignments for motor A control.
const struct {
  const int motorInputA = 15;
  const int motorInputB = 16;
} MotorAPins;

// Pin assignments for motor B control.
const struct {
  const int motorInputA = 17;
  const int motorInputB = 18;
} MotorBPins;

// Pin assignments for charger status.
const struct {
  const int chargerStatusPinA = 6;
  const int chargerStatusPinB = 5;
} ChargerPins;

// Pin assignments for any other IO.
const struct {
  const int motorDriverEnable = 4;
} OutputPins;

// Variables to store motor PWM values.
volatile int16_t motorA = 0;           // PWM value for motor A.
volatile int16_t motorB = 0;           // PWM value for motor B.
volatile uint8_t lastRequestType = 0;  // Stores the last request type received from the master.

// Represents the state of the battery charger.
enum ChargerState {
  NotConnected = 0xA0,     // Charger not connected.
  Charging = 0xA1,         // Charger is charging.
  ChargingComplete = 0xA2  // Charger has completed charging.
};

// Represents the state of the motor driver.
enum MotorDriverState {
  MotorDriverDisabled = 0xB0,  // Motor driver is disabled.
  MotorDriverEnabled = 0xB1    // Motor driver is enabled.
};

volatile ChargerState chargerState = NotConnected;                 // Variable to hold the current charger state.
volatile MotorDriverState motorDriverState = MotorDriverDisabled;  // Variable to hold the current charger state.

void setup() {
  Wire.begin(0x32);              // Initialize I2C as slave with address 0x68.
  Wire.onReceive(receiveEvent);  // Register event handler for receiving data.
  Wire.onRequest(requestEvent);  // Register event handler for sending data.

  // Pin mode definitions.
  pinMode(MotorAPins.motorInputA, OUTPUT);
  pinMode(MotorAPins.motorInputB, OUTPUT);
  pinMode(MotorBPins.motorInputA, OUTPUT);
  pinMode(MotorBPins.motorInputB, OUTPUT);
  pinMode(OutputPins.motorDriverEnable, OUTPUT);
  pinMode(ChargerPins.chargerStatusPinA, INPUT);
  pinMode(ChargerPins.chargerStatusPinB, INPUT);

  // Disable motor driver and set motor PWM values to zero on startup.
  digitalWrite(OutputPins.motorDriverEnable, LOW);
  motorA = 0;
  motorB = 0;
}

void loop() {
  // Update motor driver state by reading the motor driver enable pin logic.
  noInterrupts();  // Protect shared variables during access.
  uint8_t motorDriverEnableStatus = digitalRead(OutputPins.motorDriverEnable);
  interrupts();

  // Determine charger state based on pin readings.
  switch (motorDriverEnableStatus) {
    case 0:
      motorDriverState = MotorDriverDisabled;
      break;
    case 1:
      motorDriverState = MotorDriverEnabled;
      break;
    default:
      motorDriverState = MotorDriverDisabled;  // Fallback case.
      break;
  }

  // Update charger state by reading the charger status pins.
  noInterrupts();  // Protect shared variables during access.
  uint8_t chargerStatusA = digitalRead(ChargerPins.chargerStatusPinA);
  uint8_t chargerStatusB = digitalRead(ChargerPins.chargerStatusPinB);
  interrupts();

  // Determine charger state based on pin readings.
  switch ((chargerStatusA << 1) | chargerStatusB) {
    case 0b11:
      chargerState = NotConnected;
      break;
    case 0b01:
      chargerState = Charging;
      break;
    case 0b10:
      chargerState = ChargingComplete;
      break;
    default:
      chargerState = NotConnected;  // Fallback case.
      break;
  }

  // Handle motor control logic by checking if values have changed.
  static int16_t lastMotorA = 0;  // Store last known values for Motor A.
  static int16_t lastMotorB = 0;  // Store last known values for Motor B.

  noInterrupts();  // Protect shared variables during access.
  int16_t motorALocal = motorA;
  int16_t motorBLocal = motorB;
  interrupts();

  // Update Motor A if value has changed.
  if (motorALocal != lastMotorA) {
    controlMotorA(motorALocal);
    lastMotorA = motorALocal;  // Save new state.
  }

  // Update Motor B if value has changed.
  if (motorBLocal != lastMotorB) {
    controlMotorB(motorBLocal);
    lastMotorB = motorBLocal;  // Save new state.
  }
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
    case 0x02:  // Enable the motor driver.
      digitalWrite(OutputPins.motorDriverEnable, HIGH);
      break;
    case 0x03:  // Disable the motor driver.
      digitalWrite(OutputPins.motorDriverEnable, LOW);
      motorA = 0;
      motorB = 0;
      break;
    case 0x04:  // Set motor A PWM value.
      if (numBytes >= 3 && Wire.available() >= 2) {
        motorA = (Wire.read() << 8) | Wire.read();
      }
      break;
    case 0x05:  // Set motor B PWM value.
      if (numBytes >= 3 && Wire.available() >= 2) {
        motorB = (Wire.read() << 8) | Wire.read();
      }
      break;
    case 0x08:  // Set both motor A and motor B PWM values.
      if (numBytes >= 5 && Wire.available() >= 4) {
        motorA = (Wire.read() << 8) | Wire.read();
        motorB = (Wire.read() << 8) | Wire.read();
      }
      break;
    case 0x80:  // Test motor operation.
      testMotors();
      break;
    case 0x81:  // Wheel clean mode ENABLED.
      motorA = 255;
      motorB = 255;
      break;
    case 0x82:  // Wheel clean mode DISABLED.
      motorA = 0;
      motorB = 0;
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
      Wire.write(motorDriverState);
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
  // int speed = map(abs(value), 0, 255, 0, 255);
  int speed = abs(value);  // Direct mapping for 0-255 range.

  if (value > 0) {
    analogWrite(MotorAPins.motorInputA, speed);
    analogWrite(MotorAPins.motorInputB, 0);
  } else if (value < 0) {
    analogWrite(MotorAPins.motorInputA, 0);
    analogWrite(MotorAPins.motorInputB, speed);
  } else {
    analogWrite(MotorAPins.motorInputA, 0);
    analogWrite(MotorAPins.motorInputB, 0);
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
  // int speed = map(abs(value), 0, 255, 0, 255);
  int speed = abs(value);  // Direct mapping for 0-255 range.

  if (value > 0) {
    analogWrite(MotorBPins.motorInputA, speed);
    analogWrite(MotorBPins.motorInputB, 0);
  } else if (value < 0) {
    analogWrite(MotorBPins.motorInputA, 0);
    analogWrite(MotorBPins.motorInputB, speed);
  } else {
    analogWrite(MotorBPins.motorInputA, 0);
    analogWrite(MotorBPins.motorInputB, 0);
  }
}

/**
* Performs a motor test routine.
*/
void testMotors() {
  const int16_t MOTOR_TEST_SPEED = 80;  // Motor test speed.
  const int MOTOR_TEST_DELAY = 80;      // Delay in milliseconds for motor testing steps.

  if (digitalRead(ChargerPins.chargerStatusPinA) == HIGH && digitalRead(ChargerPins.chargerStatusPinB) == HIGH) {
    digitalWrite(OutputPins.motorDriverEnable, HIGH);
    controlMotorA(-MOTOR_TEST_SPEED);
    controlMotorB(MOTOR_TEST_SPEED);
    delay(MOTOR_TEST_DELAY);
    controlMotorA(MOTOR_TEST_SPEED);
    controlMotorB(-MOTOR_TEST_SPEED);
    delay(MOTOR_TEST_DELAY);
    controlMotorA(-MOTOR_TEST_SPEED);
    controlMotorB(MOTOR_TEST_SPEED);
    delay(MOTOR_TEST_DELAY);
    controlMotorA(MOTOR_TEST_SPEED);
    controlMotorB(-MOTOR_TEST_SPEED);
    delay(MOTOR_TEST_DELAY);
    controlMotorA(-MOTOR_TEST_SPEED);
    controlMotorB(MOTOR_TEST_SPEED);
    delay(MOTOR_TEST_DELAY);
    controlMotorA(MOTOR_TEST_SPEED);
    controlMotorB(-MOTOR_TEST_SPEED);
    delay(MOTOR_TEST_DELAY);
    controlMotorA(0);
    controlMotorB(0);
    digitalWrite(OutputPins.motorDriverEnable, LOW);
  }
}