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
* This function communicates with the I2C slave device to retrieve the current
* charger state. It returns an enumerated value representing the charger's
* state or an error if communication fails.
*
* @return ChargerState::NotConnected if the charger is not connected,
*         ChargerState::ChargingBattery if the battery is charging,
*         ChargerState::BatteryCharged if the battery is fully charged, or
*         ChargerState::Error if there is a communication failure or an unknown state.
*/
MotorDriver::ChargerState MotorDriver::getChargerState() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x01);  // Command to request the charger state.

  // Check for I2C communication failure
  if (Wire.endTransmission() != 0) {
    return ChargerState::Error;  // Return a distinct error state
  }

  delay(10);  // Small delay to ensure the request is processed

  // Check if data is available
  if (Wire.requestFrom(deviceAddress, (uint8_t)1) != 1) {
    return ChargerState::Error;  // Return a distinct error state
  }

  uint8_t state = Wire.read();

  // Map received state to ChargerState enum
  switch (state) {
    case 0xA0:
      return ChargerState::NotConnected;
    case 0xA1:
      return ChargerState::Charging;
    case 0xA2:
      return ChargerState::ChargingComplete;
    default:
      return ChargerState::Error;  // Fallback for unknown states
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
* Checks if the motor driver is enabled by communicating with the device via I2C.
*
* This function sends a command to the motor driver to request its current state
* and processes the response. The response is mapped to the `MotorDriverState` enum
* to indicate whether the motor driver is enabled, disabled, or if an error occurred.
*
* @return MotorDriverState - The current state of the motor driver:
*         - `MotorDriverState::MotorDriverDisabled` if the motor driver is disabled.
*         - `MotorDriverState::MotorDriverEnabled` if the motor driver is enabled.
*         - `MotorDriverState::Error` if there is a communication failure or an unknown state is received.
*/
MotorDriver::MotorDriverState MotorDriver::isMotorDriverEnabled() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x09);  // Command to request the charger state.

  // Check for I2C communication failure.
  if (Wire.endTransmission() != 0) {
    return MotorDriverState::Error;  // Return a distinct error state.
  }

  delay(10);  // Small delay to ensure the request is processed.

  // Check if data is available
  if (Wire.requestFrom(deviceAddress, (uint8_t)1) != 1) {
    return MotorDriverState::Error;  // Return a distinct error state.
  }

  uint8_t state = Wire.read();

  // Map received state to ChargerState enum.
  switch (state) {
    case 0xB0:
      return MotorDriverState::MotorDriverDisabled;
    case 0xB1:
      return MotorDriverState::MotorDriverEnabled;
    default:
      return MotorDriverState::Error;  // Fallback for unknown states.
  }
}

/**
* Sends a command to the I2C slave to initiate a motor test routine.
*
* This function triggers a predefined motor test sequence on the I2C slave device.
*
* @return True if the command was successfully sent to the slave device; false otherwise.
*/
bool MotorDriver::commitMotorTest() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x80);                      // Command to disable the motor driver.
  return (Wire.endTransmission() == 0);  // Return true on success.
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
  Wire.write(0x08);                      // Command to set both motor values.
  Wire.write((uint8_t)(motorA >> 8));    // High byte of motorA.
  Wire.write((uint8_t)motorA);           // Low byte of motorA.
  Wire.write((uint8_t)(motorB >> 8));    // High byte of motorB.
  Wire.write((uint8_t)motorB);           // Low byte of motorB.
  return (Wire.endTransmission() == 0);  // Return true on success.
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

/**
* Enables the wheel cleaning mode by sending a command to the I2C slave.
* This function sends a specific command to the motor driver to enable wheel cleaning mode.
*
* @return True if the I2C transmission was successful, false otherwise.
*/
bool MotorDriver::enableWheelCleanMode() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x81);                      // Command to enable the motor driver.
  return (Wire.endTransmission() == 0);  // Return true on success.
}

/**
* Disables the wheel cleaning mode by sending a command to the I2C slave.
* This function sends a specific command to the motor driver to disable wheel cleaning mode.
*
* @return True if the I2C transmission was successful, false otherwise.
*/
bool MotorDriver::disableWheelCleanMode() {
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x82);                      // Command to enable the motor driver.
  return (Wire.endTransmission() == 0);  // Return true on success.
}