/**
* MotorDriver.h
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

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include "Arduino.h"
#include <Wire.h>

class MotorDriver {
public:
  /**
  * Represents the state of the battery charger.
  *
  * This enum defines the possible states of the charger, including whether it is 
  * connected, charging, fully charged, or an error occurred.
  */
  enum class ChargerState : uint8_t {
    NotConnected = 0xA0,
    Charging = 0xA1,
    ChargingComplete = 0xA2,
    Error = 0
  };

  // Represents the state of the motor driver.
  enum class MotorDriverState : uint8_t {
    MotorDriverDisabled = 0xB0,
    MotorDriverEnabled = 0xB1,
    Error = 0
  };

  /**
  * Constructor for the MotorDriver class.
  * 
  * @param address The I2C address of the device.
  */
  MotorDriver(uint8_t address);

  /**
  * Initializes the I2C communication and checks if the device is connected.
  * 
  * @return True if the device is found, false otherwise.
  */
  bool begin();

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
  ChargerState getChargerState();

  /**
  * Sends a command to the I2C slave to enable the motor driver.
  * 
  * @return True if the transmission was successful, false otherwise.
  */
  bool enableMotorDriver();

  /**
  * Sends a command to the I2C slave to disable the motor driver.
  * 
  * @return True if the transmission was successful, false otherwise.
  */
  bool disableMotorDriver();

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
  MotorDriverState isMotorDriverEnabled();

  /**
  * Sends a command to the I2C slave to initiate a motor test routine.
  *
  * This function triggers a predefined motor test sequence on the I2C slave device.
  *
  * @return True if the command was successfully sent to the slave device; false otherwise.
  */
  bool commitMotorTest();

  /**
  * Sends the PWM value for motor A to the I2C slave.
  * 
  * @param motorA The PWM value for motor A, can range from -255 to 255.
  * @return True if the transmission was successful, false otherwise.
  */
  bool setMotorA(int16_t motorA);

  /**
  * Sends the PWM value for motor B to the I2C slave.
  * 
  * @param motorB The PWM value for motor B, can range from -255 to 255.
  * @return True if the transmission was successful, false otherwise.
  */
  bool setMotorB(int16_t motorB);

  /**
  * Sends the PWM values for both motor A and motor B to the I2C slave.
  * 
  * @param motorA The PWM value for motor A, can range from -255 to 255.
  * @param motorB The PWM value for motor B, can range from -255 to 255.
  * @return True if the transmission was successful, false otherwise.
  */
  bool setMotorValues(int16_t motorA, int16_t motorB);

  /**
  * Requests the PWM value for motor A from the I2C slave.
  * 
  * @return The 16-bit PWM value for motor A, or 0 on error.
  */
  int16_t getMotorAValue();

  /**
  * Requests the PWM value for motor B from the I2C slave.
  * 
  * @return The 16-bit PWM value for motor B, or 0 on error.
  */
  int16_t getMotorBValue();

  /**
  * Enables the wheel cleaning mode by sending a command to the I2C slave.
  * This function sends a specific command to the motor driver to enable wheel cleaning mode.
  *
  * @return True if the I2C transmission was successful, false otherwise.
  */
  bool enableWheelCleanMode();

  /**
  * Disables the wheel cleaning mode by sending a command to the I2C slave.
  * This function sends a specific command to the motor driver to disable wheel cleaning mode.
  *
  * @return True if the I2C transmission was successful, false otherwise.
  */
  bool disableWheelCleanMode();
private:
  uint8_t deviceAddress;  // The I2C address of the slave device.
};

#endif  // MOTORDRIVER_H