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

const int sda = 1;       // IO01, SDA
const int scl = 2;       // IO02, SCL
const int neoPixel = 4;  // IO04, ADC1-CH0
const int speaker = 5;   // IO05, ADC1-CH4

AsyncWebServer server(80);                 // Create AsyncWebServer object on port 80.
AsyncWebSocket webSocket("/ws");           // Create a WebSocket object.
const char* apName = "SMAF-DK-Mouse-Bot";  // AP name.
const char* apPassword = "12345678";       // AP password.

SFE_MAX1704X fuelGauge(MAX1704X_MAX17048);  // Create a MAX17048.
MotorDriver motorDriver(0x68);              // Create a MotorDriver. Replace 0x68 with your slave device's address.

/**
* Constructs an AudioVisualNotifications object with specified parameters.
* Initializes the NeoPixel and speaker pin settings for audio-visual notifications.
* 
* @param neoPixelPin The GPIO pin connected to the NeoPixel.
* @param neoPixelCount The number of NeoPixels in the strip.
* @param neoPixelBrightness The brightness level of the NeoPixels (0-255).
* @param speakerPin The GPIO pin connected to the speaker.
*/
AudioVisualNotifications notifications(neoPixel, 2, 40, speaker);

// Define constants for ESP32 core numbers.
#define ESP32_CORE_PRIMARY 0    // Numeric value representing the primary core.
#define ESP32_CORE_SECONDARY 1  // Numeric value representing the secondary core.

// Function prototype for the DeviceStatusThread function.
void DeviceStatusThread(void* pvParameters);

/**
* Enum representing the different statuses of battery charging.
* This enumeration indicates the current state of the battery charge.
*/
enum BatteryChargeStatusEnum : byte {
  NONE,      // Charger not connected.
  CHARGING,  // Charger connected and charging battery.
  CHARGED,   // Charger connected and battery is charged.
};

float batteryVoltage = 0.0;                          // Variable to keep track of battery voltage.
int batterySoC = 0;                                  // Variable to keep track of battery state-of-charge (SOC).
BatteryChargeStatusEnum batteryChargeStatus = NONE;  // Variable to store the current charge status.

// Function to handle 404 errors
void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}

/**
* This function is responsible for the initial setup of the SMAF-DK-Mousebot.
* It is executed once when the Arduino board starts or is reset.
*/
void setup() {
  // Create a new task (DeviceStatusThread) and assign it to the primary core (ESP32_CORE_PRIMARY).
  xTaskCreatePinnedToCore(
    DeviceStatusThread,    // Function to implement the task.
    "DeviceStatusThread",  // Name of the task.
    8000,                  // Stack size in words.
    NULL,                  // Task input parameter (e.g., delay).
    1,                     // Priority of the task.
    NULL,                  // Task handle.
    ESP32_CORE_SECONDARY   // Core where the task should run.
  );

  Serial.begin(115200);

  // Initialize visualization library neo pixels.
  notifications.initializeVisualNotifications();
  notifications.clearAllVisualNotifications();
  notifications.initializePixel(0, 0, 0, 255);  // Turn blue RGB led on.
  notifications.initializePixel(1, 0, 0, 255);  // Turn blue RGB led on.

  // Play intro melody on speaker.
  notifications.introAudioNotification();

  // Give ESP32 core some time to initialize all.
  delay(2400);

  // Set Wire library custom I2C pins.
  // Example usage:
  // Wire.setPins(SDA_PIN_NUMBER, SCL_PIN_NUMBER);
  Wire.setPins(sda, scl);
  Wire.begin();

  // Initialize the fuel gauge.
  if (fuelGauge.begin() == false) {
    debug(ERR, "Fuel gauge not found on I2C bus. Check wiring.");
    notifications.initializePixel(0, 255, 0, 0);  // Turn blue RGB led on.
    notifications.initializePixel(1, 255, 0, 0);  // Turn blue RGB led on.
    while (true) {}
  }

  // Initialize the motor driver.
  if (motorDriver.begin() == false) {
    debug(ERR, "Motor driver not found on I2C bus. Check wiring.");
    notifications.initializePixel(0, 255, 0, 0);  // Turn blue RGB led on.
    notifications.initializePixel(1, 255, 0, 0);  // Turn blue RGB led on.
    while (true) {}
  }

  // Check motors.
  if (motorDriver.getChargerState() == 1) {
    int value = 80;

    motorDriver.enableMotorDriver();
    motorDriver.setMotorValues(value, -value);
    delay(80);
    motorDriver.setMotorValues(-value, value);
    delay(80);
    motorDriver.setMotorValues(value, -value);
    delay(80);
    motorDriver.setMotorValues(-value, value);
    delay(80);
    motorDriver.setMotorValues(value, -value);
    delay(80);
    motorDriver.setMotorValues(-value, value);
    delay(80);
    motorDriver.setMotorValues(0, 0);
    motorDriver.disableMotorDriver();
  }

  // Quick start restarts the fuel gauge in hopes of getting a more accurate.
  fuelGauge.quickStart();

  // Initialize softAP and print IP address in terminal when softAP is initialized.
  WiFi.softAP(apName, apPassword);
  debug(SCS, "AP IP address: %s", String(WiFi.softAPIP()).c_str());

  // Initialize LittleFS.
  if (!LittleFS.begin()) {
    debug(ERR, "An error occurred while mounting LittleFS.");
    return;
  }

  // Serve the HTML page.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index.html", String(), false);
  });

  // Serve static files for web page.
  server.serveStatic("/style.css", LittleFS, "/style.css");
  server.serveStatic("/script.js", LittleFS, "/script.js");
  server.serveStatic("/fonts/albert-sans.ttf", LittleFS, "/fonts/albert-sans.ttf");

  // WebSocket event handler.
  webSocket.onEvent(onWebSocketEvent);
  server.addHandler(&webSocket);

  // Handle 404 errors.
  server.onNotFound(notFound);

  // Start server and show success message in terminal.
  server.begin();
  debug(SCS, "Web server started. You can control the Mousebot.");
}

/**
* This function runs repeatedly in a loop after the initial setup.
* It is the core of your Arduino program, where continuous tasks and operations should be placed.
* Be mindful of keeping the loop efficient and avoiding long blocking operations.
*/
void loop() {
  // Slow things down a bit.
  delay(240);

  // Request and print the charger state from the slave
  uint8_t chargerState = motorDriver.getChargerState();

  if (chargerState) {
    switch (chargerState) {
      case 1:
        batteryChargeStatus = NONE;
        motorDriver.enableMotorDriver();
        notifications.initializePixel(0, 0, 0, 255);
        break;
      case 2:
        batteryChargeStatus = CHARGING;
        motorDriver.disableMotorDriver();
        notifications.initializePixel(0, 255, 0, 0);
        break;
      case 3:
        batteryChargeStatus = CHARGED;
        motorDriver.disableMotorDriver();
        notifications.initializePixel(0, 0, 255, 0);
        break;
      default:
        Serial.println("Charger state: Unknown");
        break;
    }
  }

  // Write all fuel gauge values to variables.
  batteryVoltage = fuelGauge.getVoltage();
  batterySoC = fuelGauge.getSOC();

  // Fuel gauge sometimes can show SoC larger than 100% so this prevents that.
  if (batterySoC > 100) {
    batterySoC = 100;
  }

  // Rewrite enum for charger status to show pretty print strings in terminal.
  String chargerStatus = (batteryChargeStatus == NONE) ? "Running on battery" : (batteryChargeStatus == CHARGING) ? "Battery charging"
                                                                                                                  : "Battery charged";

  // Show status for battery and charger in terminal.
  debug(LOG, "Battery state - %sV, %s%%, %s.", String(batteryVoltage).c_str(), String(batterySoC).c_str(), String(chargerStatus).c_str());

  // Send battery data to web socket.
  sendBatteryDataToWebSocket(batteryVoltage, batterySoC, batteryChargeStatus);

  // Quick start restarts the fuel gauge in hopes of getting a more accurate guess for the SOC.
  fuelGauge.quickStart();
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
      if (len == 4) {  // 4 bytes expected (2 bytes for each motor value).
        // Extract the motor values from the data array.
        int16_t rightMotorPWMValue = (int16_t)(data[0] | (data[1] << 8));  // Little-endian.
        int16_t leftMotorPWMValue = (int16_t)(data[2] | (data[3] << 8));   // Little-endian.

        // Log received motor PWM values.
        debug(LOG, "Motor PWM values - Right motor at %s, Left at motor %s.", String(rightMotorPWMValue), String(leftMotorPWMValue));

        // Use the values to control the motors.
        motorDriver.setMotorValues(-leftMotorPWMValue, -rightMotorPWMValue);
      } else {
        debug(LOG, "Unexpected binary packet length.");
      }
    }
  }
}

/**
* Sends battery data to connected WebSocket clients.
* This function packages the battery voltage, state of charge (SoC), and charger status into a binary packet.
* 
* @param batteryVoltage The current battery voltage.
* @param batterySoC The current state of charge of the battery (percentage).
* @param chargerStatus The current status of the charger (enum).
*/
void sendBatteryDataToWebSocket(float batteryVoltage, int batterySoC, BatteryChargeStatusEnum chargerStatus) {
  uint8_t packet[6];

  memcpy(&packet[0], &batteryVoltage, sizeof(float));      // Battery Voltage (float, 4 bytes).
  packet[4] = batterySoC;                                  // Battery SoC (1 byte).
  packet[5] = static_cast<uint8_t>(chargerStatus) & 0x03;  // Charge Status (1 byte, with 2 least significant bits used). Ensure only 2 bits are used for the status.

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
  while (true) {

    // Add a delay to prevent WDT timeout.
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for 10 milliseconds.
  }
}