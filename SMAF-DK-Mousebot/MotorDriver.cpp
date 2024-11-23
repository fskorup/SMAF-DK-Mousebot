/**
* MotorDriver.cpp
* Declaration of the MotorDriver library for motor and battery charge control.
*
* This file contains the declaration for the MotorDriver library, which facilitates
* the indication of device charging status and motor control through a SAMD21G18A as a I2C slave device.
* It is designed to be easily integrated into Arduino projects for visualizing various device states.
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

#include "MotorDriver.h"

/**
* Constructor for the MotorDriver class.
* 
* @param address The I2C address of the device.
*/
MotorDriver::MotorDriver(uint8_t address)
  : deviceAddress(address) {}

/**
* Initializes the I2C communication and checks if the device is connected.
* 
* @return True if the device is found, false otherwise.
*/
bool MotorDriver::begin() {
  Wire.beginTransmission(deviceAddress);
  return (Wire.endTransmission() == 0);  // 0 means success (device is found).
}

/**
* Requests the state of the battery charger from the I2C slave device.
* 
* @return The charger state (0 for not connected, 1 for charging, 2 for complete, or 0 for error).
*/
uint8_t MotorDriver::getChargerState() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x01);                           // Command to request the charger state.
  if (Wire.endTransmission() != 0) return 0;  // Return 0 on error.

  delay(10);  // Small delay to ensure the request is processed.

  Wire.requestFrom(deviceAddress, (uint8_t)1);  // Request 1 byte from the slave.
  if (Wire.available()) {
    return Wire.read();
  } else {
    return 0;  // Return 0 on error.
  }
}

/**
* Sends a command to the I2C slave to enable the motor driver.
* 
* @return True if the transmission was successful, false otherwise.
*/
bool MotorDriver::enableMotorDriver() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x02);                      // Command to enable the motor driver.
  return (Wire.endTransmission() == 0);  // Return true on success.
}

/**
* Sends a command to the I2C slave to disable the motor driver.
* 
* @return True if the transmission was successful, false otherwise.
*/
bool MotorDriver::disableMotorDriver() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x03);                      // Command to disable the motor driver.
  return (Wire.endTransmission() == 0);  // Return true on success.
}

/**
* Checks if the motor driver is enabled on the I2C slave.
* 
* @return 1 if the motor driver is enabled, 2 if disabled, or 0 on failure.
*/
uint8_t MotorDriver::isMotorDriverEnabled() {
    Wire.beginTransmission(deviceAddress);
    Wire.write(0x09); // Command to request motor driver enabled state.
    if (Wire.endTransmission() != 0) return 0; // Return 0 on transmission error.

    delay(10); // Small delay to ensure the request is processed.

    Wire.requestFrom(deviceAddress, (uint8_t)1); // Request 1 byte from the slave.
    if (Wire.available() == 1) {
        uint8_t response = Wire.read();
        return (response == 1 || response == 2) ? response : 0; // Validate response.
    } else {
        return 0; // Return 0 if no data is received.
    }
}

/**
* Sends the PWM value for motor A to the I2C slave.
* 
* @param motorA The PWM value for motor A, can range from -255 to 255.
* @return True if the transmission was successful, false otherwise.
*/
bool MotorDriver::setMotorA(int16_t motorA) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x04);                      // Command identifier for setting motor A.
  Wire.write((uint8_t)(motorA >> 8));    // High byte of motorA
  Wire.write((uint8_t)motorA);           // Low byte of motorA
  return (Wire.endTransmission() == 0);  // Return true on success
}

/**
* Sends the PWM value for motor B to the I2C slave.
* 
* @param motorB The PWM value for motor B, can range from -255 to 255.
* @return True if the transmission was successful, false otherwise.
*/
bool MotorDriver::setMotorB(int16_t motorB) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x05);                      // Command identifier for setting motor B
  Wire.write((uint8_t)(motorB >> 8));    // High byte of motorB.
  Wire.write((uint8_t)motorB);           // Low byte of motorB.
  return (Wire.endTransmission() == 0);  // Return true on success.
}

/**
* Sends the PWM values for both motor A and motor B to the I2C slave.
* 
* @param motorA The PWM value for motor A, can range from -255 to 255.
* @param motorB The PWM value for motor B, can range from -255 to 255.
* @return True if the transmission was successful, false otherwise.
*/
bool MotorDriver::setMotorValues(int16_t motorA, int16_t motorB) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(0x08); // Command to set both motor values.
    Wire.write((uint8_t)(motorA >> 8)); // High byte of motorA.
    Wire.write((uint8_t)motorA);        // Low byte of motorA.
    Wire.write((uint8_t)(motorB >> 8)); // High byte of motorB.
    Wire.write((uint8_t)motorB);        // Low byte of motorB.
    return (Wire.endTransmission() == 0); // Return true on success.
}

/**
* Requests the PWM value for motor A from the I2C slave.
* 
* @return The 16-bit PWM value for motor A, or 0 on error.
*/
int16_t MotorDriver::getMotorAValue() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x06);                           // Command to request motor A value.
  if (Wire.endTransmission() != 0) return 0;  // Return 0 on error.

  delay(10);  // Small delay to ensure the request is processed.

  Wire.requestFrom(deviceAddress, (uint8_t)2);  // Request 2 bytes from the slave.
  if (Wire.available() == 2) {
    int16_t highByte = Wire.read();
    int16_t lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  } else {
    return 0;  // Return 0 on error.
  }
}

/**
* Requests the PWM value for motor B from the I2C slave.
* 
* @return The 16-bit PWM value for motor B, or 0 on error.
*/
int16_t MotorDriver::getMotorBValue() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x07);                           // Command to request motor B value.
  if (Wire.endTransmission() != 0) return 0;  // Return 0 on error.

  delay(10);  // Small delay to ensure the request is processed.

  Wire.requestFrom(deviceAddress, (uint8_t)2);  // Request 2 bytes from the slave.
  if (Wire.available() == 2) {
    int16_t highByte = Wire.read();
    int16_t lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  } else {
    return 0;  // Return 0 on error.
  }
}