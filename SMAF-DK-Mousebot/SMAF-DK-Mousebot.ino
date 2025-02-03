/**
* SMAF-DK-Mousebot.ino
* Main Arduino sketch for the SMAF-DK-Mousebot project.
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

#include "WiFi.h"
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "LittleFS.h"
#include "Helpers.h"
#include "AudioVisualNotifications.h"
#include "MotorDriver.h"
#include "SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h"
#include "Arduino_BMI270_BMM150.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SquareEyes.h"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin assignments for I2C.
const struct {
  const int SDA = 1;  // IO01, SDA
  const int SCL = 2;  // IO02, SCL
} I2CPins;

// Pin assignments for outputs.
const struct {
  const int NeoPixel = 4;  // IO04, ADC1-CH0
  const int Speaker = 5;   // IO05, ADC1-CH4
} OutputPins;

// Pin assignments for inputs.
const struct {
  const int UserButton = 6;  // IO06
} InputPins;

// Enum to represent different device statuses.
enum BotStatusEnum : byte {
  NONE,
  NOT_READY,
  CHARGING,
  CHARGING_COMPLETE,
  READY_TO_DRIVE,
  WHEEL_CLEAN_MODE
};

// Variable to store the current device status.
BotStatusEnum botStatus = NONE;  // Initial state is set to NOT_READY.

AsyncWebServer server(80);                 // Create AsyncWebServer object on port 80.
AsyncWebSocket webSocket("/ws");           // Create a WebSocket object.
const char* apName = "SMAF-DK-Mouse-Bot";  // AP name.
const char* apPassword = "12345678";       // AP password.
const char* ssid = "SSID";                 // Home network AP name.
const char* password = "PASS";             // Home network AP password.

// I2C devices definitions.
SFE_MAX1704X fuelGauge(MAX1704X_MAX17048);  // Create a MAX17048.
MotorDriver motorDriver(0x32);              // Create a MotorDriver. Replace 0x68 with your slave device's address.
SquareEyes eyes(display);                   // Eye width, height, and spacing.

/**
* Constructs an AudioVisualNotifications object with specified parameters.
* Initializes the NeoPixel and speaker pin settings for audio-visual notifications.
* 
* @param neoPixelPin The GPIO pin connected to the NeoPixel.
* @param neoPixelCount The number of NeoPixels in the strip.
* @param neoPixelBrightness The brightness level of the NeoPixels (0-255).
* @param speakerPin The GPIO pin connected to the speaker.
*/
AudioVisualNotifications notifications(OutputPins.NeoPixel, 2, 40, OutputPins.Speaker);

// Define constants for ESP32 core numbers.
#define ESP32_CORE_PRIMARY 0    // Numeric value representing the primary core.
#define ESP32_CORE_SECONDARY 1  // Numeric value representing the secondary core.

// Function prototype for the DeviceStatusThread function.
void DeviceStatusThread(void* pvParameters);

// Enum variables.
MotorDriver::ChargerState chargerState = MotorDriver::ChargerState::NotConnected;                     // Variable to store the current charger status.
MotorDriver::MotorDriverState motorDriverState = MotorDriver::MotorDriverState::MotorDriverDisabled;  // Variable to store the current motor driver status.

int lastUserButtonState = HIGH;  // Previous button state.
bool wheelCleanMode = false;     // Variable to keep track wheel clean mode.
float accX, accY, accZ;          // Accelerometer readings.

// Function to handle 404 errors
void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}

/**
* This function is responsible for the initial setup of the SMAF-DK-Mousebot.
* It is executed once when the Arduino board starts or is reset.
*/
void setup() {
  Serial.begin(115200);

  // Create a new task (DeviceStatusThread) and assign it to the primary core (ESP32_CORE_PRIMARY).
  xTaskCreatePinnedToCore(
    DeviceStatusThread,    // Function to implement the task.
    "DeviceStatusThread",  // Name of the task.
    8000,                  // Stack size in words.
    NULL,                  // Task input parameter (e.g., delay).
    1,                     // Priority of the task.
    NULL,                  // Task handle.
    ESP32_CORE_PRIMARY     // Core where the task should run.
  );

  // Set the pin mode for the user button to INPUT.
  pinMode(InputPins.UserButton, INPUT);

  // Set Wire library custom I2C pins.
  // Example usage:
  // Wire.setPins(SDA_PIN_NUMBER, SCL_PIN_NUMBER);
  Wire.setPins(I2CPins.SDA, I2CPins.SCL);
  Wire.begin();

  while (IMU.begin() == false) {
    debug(ERR, "IMU not found on I2C bus. Check wiring.");
    delay(240);
  }

  // Initialize the fuel gauge.
  while (fuelGauge.begin() == false) {
    debug(ERR, "Fuel gauge not found on I2C bus. Check wiring.");
    delay(240);
  }

  // Initialize the motor driver.
  while (motorDriver.begin() == false) {
    debug(ERR, "Motor driver not found on I2C bus. Check wiring.");
    delay(240);
  }

  // If motor driver initialised, disable it and set motor PWM values to zero.
  motorDriver.disableMotorDriver();
  motorDriver.setMotorValues(0, 0);

  while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    debug(ERR, "OLED not found on I2C bus. Check wiring.");
    delay(240);
  }

  eyes.begin();
  eyes.setBlinkIntervalRange(800, 3200);  // Blink interval between 1 and 5 seconds.
  eyes.setBlinkDuration(40);              // Blink lasts for 32 ms.

  // Initialize softAP and print IP address in terminal when softAP is initialized.
  WiFi.softAP(apName, apPassword);
  // debug(SCS, "AP IP address: %s", String(WiFi.softAPIP()).c_str());

  // WiFi.begin(ssid, password);

  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }

  // Serial.println("");

  // // Print local IP address and start web server.
  // Serial.println("");
  // Serial.println("WiFi connected.");
  // Serial.println("IP address: ");
  // Serial.println(WiFi.localIP());

  delay(2400);

  // debug(SCS, "AP IP address: %s", WiFi.localIP().c_str());

  // Initialize LittleFS.
  // if (!LittleFS.begin()) {
  //   debug(ERR, "An error occurred while mounting LittleFS.");
  //   return;
  // }

  // Serve the HTML page.
  // server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
  //   request->send(LittleFS, "/index.html", String(), false);
  // });

  // Serve static files for web page.
  // server.serveStatic("/style.css", LittleFS, "/style.css");
  // server.serveStatic("/script.js", LittleFS, "/script.js");
  // server.serveStatic("/fonts/albert-sans.ttf", LittleFS, "/fonts/albert-sans.ttf");

  // WebSocket event handler.
  webSocket.onEvent(onWebSocketEvent);
  server.addHandler(&webSocket);

  // Increase the default HTTP session timeout
  // DefaultHeaders::Instance().addHeader("Connection", "keep-alive");

  // Handle 404 errors.
  // server.onNotFound(notFound);

  // Start server and show success message in terminal.
  server.begin();
  debug(SCS, "Web server started. You can control the Mousebot.");

  // Quick start restarts the fuel gauge in hopes of getting a more accurate.
  fuelGauge.quickStart();

  // Test motors.
  eyes.idle();  // Enter idle mode.
  motorDriver.commitMotorTest();
}

/**
* This function runs repeatedly in a loop after the initial setup.
* It is the core of your Arduino program, where continuous tasks and operations should be placed.
* Be mindful of keeping the loop efficient and avoiding long blocking operations.
*/
void loop() {
  eyes.update();

  // Slow things down a bit.
  delay(40);

  // Read accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
  }

  (accZ > 0) ? (eyes.flipVertical(true)) : (eyes.flipVertical(false));

  // debug(LOG, "IMU: X %s, Y %s, Z %s.", String(accX).c_str(), String(accY).c_str(), String(accZ).c_str());

  // Set static variables that are only used in this loop function. Static variables are constructed only once.
  static MotorDriver::ChargerState chargerStateRaw;
  static MotorDriver::MotorDriverState motorDriverStateRaw;
  static String chargerStateString = String();
  static String motorDriverStateString = String();

  chargerStateRaw = motorDriver.getChargerState();
  motorDriverStateRaw = motorDriver.isMotorDriverEnabled();

  if (chargerStateRaw != MotorDriver::ChargerState::Error) {
    chargerState = chargerStateRaw;

    switch (chargerState) {
      case (MotorDriver::ChargerState::NotConnected):
        chargerStateString = "Running on battery";
        motorDriver.enableMotorDriver();
        toggleWheelCleanMode(accZ);

        if (wheelCleanMode) {
          botStatus = WHEEL_CLEAN_MODE;
          eyes.cleanMode();
        } else {
          botStatus = READY_TO_DRIVE;
          eyes.idle();
        }

        break;
      case (MotorDriver::ChargerState::Charging):
        chargerStateString = "Battery charging";
        motorDriver.disableMotorDriver();
        resetWheelCleanMode();
        eyes.charging();

        botStatus = CHARGING;

        break;
      case (MotorDriver::ChargerState::ChargingComplete):
        chargerStateString = "Battery charged";
        motorDriver.disableMotorDriver();
        resetWheelCleanMode();
        eyes.chargingComplete();

        botStatus = CHARGING_COMPLETE;

        break;
      default:
        break;
    }
  }

  if (motorDriverStateRaw != MotorDriver::MotorDriverState::Error) {
    motorDriverState = motorDriverStateRaw;

    switch (motorDriverState) {
      case (MotorDriver::MotorDriverState::MotorDriverDisabled):
        motorDriverStateString = "Motor driver disabled";
        break;
      case (MotorDriver::MotorDriverState::MotorDriverEnabled):
        motorDriverStateString = "Motor driver enabled";
        break;
      default:
        break;
    }
  }

  // Write all fuel gauge values to variables.
  fuelGauge.quickStart();                                  // Quick start restarts the fuel gauge in hopes of getting a more accurate guess for the SOC.
  float batteryVoltage = fuelGauge.getVoltage();           // Get battery voltage.
  int batterySoC = constrain(fuelGauge.getSOC(), 0, 100);  // Constrain the value to a range from 0 to 100.

  // Send battery data to web socket and show status for battery and charger in terminal.
  // debug(LOG, "Battery state: %sV, %s%%, %s, %s.", String(batteryVoltage).c_str(), String(batterySoC).c_str(), String(chargerStateString).c_str(), String(motorDriverStateString).c_str());
  sendBatteryDataToWebSocket(batteryVoltage, batterySoC, chargerState, accX, accY, accZ);
}

/**
* Toggles the wheel clean mode when the user button is pressed. 
* If the mode is enabled, the motor driver is updated accordingly.
*/
void toggleWheelCleanMode(float accZ) {
  int currentUserButtonState = digitalRead(InputPins.UserButton);

  if (currentUserButtonState == LOW && lastUserButtonState == HIGH) {
    if (accZ > 0) {
      wheelCleanMode = !wheelCleanMode;
      if (wheelCleanMode) {
        eyes.cleanMode();
        notifications.audio.beep();
        motorDriver.enableWheelCleanMode();
      } else {
        eyes.idle();
        notifications.audio.doubleBeep();
        motorDriver.disableWheelCleanMode();
      }
    } else {
      notifications.audio.tripleBeep();
    }
  }

  lastUserButtonState = currentUserButtonState;
}

/**
* Resets the wheel clean mode to its default state (disabled).
* Sets the `wheelCleanMode` variable to false.
*/
void resetWheelCleanMode() {
  wheelCleanMode = false;
}

/**
* Handles WebSocket events for client connections and data reception.
* This function processes connection and disconnection events, as well as incoming binary data.
* 
* @param server The WebSocket server instance.
* @param client The client instance that triggered the event.
* @param type The type of the event (connect, disconnect, data).
* @param arg Additional argument for event handling.
* @param data Pointer to the received data.
* @param len Length of the received data.
*/
void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    debug(LOG, "Client connected.");
  } else if (type == WS_EVT_DISCONNECT) {
    debug(LOG, "Client disconnected.");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;

    // Ensure we received all the data in one packet and it's binary data
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_BINARY) {
      // Process based on the command type
      uint8_t commandType = data[0];                                       // First byte indicates the command type
      if (commandType == 0x01 && len == 5) {                               // Motor control command
        int16_t rightMotorPWMValue = (int16_t)(data[1] | (data[2] << 8));  // Little-endian
        int16_t leftMotorPWMValue = (int16_t)(data[3] | (data[4] << 8));   // Little-endian

        debug(LOG, "Motor PWM values - Right motor at %s, Left motor at %s.", String(rightMotorPWMValue), String(leftMotorPWMValue));

        // Use the values to control the motors
        motorDriver.setMotorValues(leftMotorPWMValue, rightMotorPWMValue);
      } else if (commandType == 0x02 && len == 2) {  // Example: Third command
        uint8_t customValue = data[1];               // Example payload (e.g., configuration setting)

        // Implement custom behavior here
        // toggleWheelCleanMode(accZ);

        if (accZ > 0) {
          wheelCleanMode = !wheelCleanMode;
          if (wheelCleanMode) {
            eyes.cleanMode();
            notifications.audio.beep();
            motorDriver.enableWheelCleanMode();
          } else {
            eyes.idle();
            notifications.audio.doubleBeep();
            motorDriver.disableWheelCleanMode();
          }
        } else {
          notifications.audio.tripleBeep();
        }
      } else {
        debug(LOG, "Unexpected binary packet or command type.");
      }
    }
  }
}

/**
* Sends battery and IMU data to connected WebSocket clients.
* This function packages the battery voltage, state of charge (SoC), charger status,
* and filtered angles gx and gy into a binary packet.
* 
* @param batteryVoltage The current battery voltage.
* @param batterySoC The current state of charge of the battery (percentage).
* @param chargerStatus The current status of the charger (MotorDriver::ChargerState enum).
* @param accX The filtered angle around the X-axis.
* @param accY The filtered angle around the Y-axis.
* @param accZ The filtered angle around the Z-axis.
*/
void sendBatteryDataToWebSocket(float batteryVoltage, int batterySoC, MotorDriver::ChargerState chargerStatus, float accX, float accY, float accZ) {
  // Define the packet size: 4 bytes for float, 1 byte for batterySoC, 1 byte for chargerStatus,
  // 4 bytes for gx, and 4 bytes for gy. Total = 14 bytes.
  uint8_t packet[18];

  // Serialize batteryVoltage into bytes 0-3.
  memcpy(&packet[0], &batteryVoltage, sizeof(float));  // 4 bytes

  // Serialize batterySoC into byte 4.
  packet[4] = static_cast<uint8_t>(batterySoC);  // 1 byte

  // Serialize chargerStatus into byte 5.
  packet[5] = static_cast<uint8_t>(chargerStatus);  // 1 byte

  // Serialize gx into bytes 6-9.
  memcpy(&packet[6], &accX, sizeof(float));  // 4 bytes

  // Serialize gy into bytes 10-13.
  memcpy(&packet[10], &accY, sizeof(float));  // 4 bytes

  // Serialize gy into bytes 10-13.
  memcpy(&packet[14], &accZ, sizeof(float));  // 4 bytes

  // Send the packet over WebSocket.
  webSocket.binaryAll(packet, sizeof(packet));
}

/**
* Continuously runs to monitor device status.
* This function includes a delay to prevent the watchdog timer from timing out.
* 
* @param pvParameters Pointer to parameters passed to the task (not used in this implementation).
*/
void DeviceStatusThread(void* pvParameters) {
  // Initialize notifications.
  notifications.visual.initializePixels();
  notifications.visual.clearAllPixels();
  notifications.audio.introMelody();

  while (true) {
    // Update LED status based on the current device status.
    switch (botStatus) {
      case NONE:
        notifications.visual.notReadyMode();
        break;
      case NOT_READY:
        notifications.visual.notReadyMode();
        break;
      case CHARGING:
        notifications.visual.singlePixel(0, 255, 0, 0);
        notifications.visual.singlePixel(1, 255, 0, 0);
        break;
      case CHARGING_COMPLETE:
        notifications.visual.singlePixel(0, 0, 255, 0);
        notifications.visual.singlePixel(1, 255, 0, 0);
        break;
      case READY_TO_DRIVE:
        notifications.visual.singlePixel(0, 0, 0, 255);
        notifications.visual.singlePixel(1, 0, 0, 255);
        break;
      case WHEEL_CLEAN_MODE:
        notifications.visual.singlePixel(0, 255, 0, 255);
        notifications.visual.singlePixel(1, 255, 0, 255);
        break;
    }

    // Add a delay to prevent WDT timeout.
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for 10 milliseconds.
  }
}