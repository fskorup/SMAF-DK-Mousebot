#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Wire.h>

/**
 * MotorDriver class for managing I2C communication for motor control and charger state retrieval.
 */
class MotorDriver {
public:
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
     * @return The charger state (0 for not connected, 1 for charging, 2 for complete, or 0 for error).
     */
  uint8_t getChargerState();

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
  * Checks if the motor driver is enabled on the I2C slave.
  * 
  * @return 1 if the motor driver is enabled, 2 if disabled, or 0 on failure.
  */
  uint8_t isMotorDriverEnabled();

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

private:
  uint8_t deviceAddress;  ///< The I2C address of the slave device.
};

#endif  // MOTORDRIVER_H