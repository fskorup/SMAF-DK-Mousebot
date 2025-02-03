#include "SquareEyes.h"
#include <stdlib.h>  // For random()

// Constructor
SquareEyes::SquareEyes(Adafruit_SSD1306 &display)
  : display(display),
    lastBlinkTime(0),
    minBlinkInterval(2000),  // Default minimum blink interval
    maxBlinkInterval(5000),  // Default maximum blink interval
    blinkDuration(200),      // Default blink duration
    isStaticBitmap(false),
    isBlinking(false),
    lastIdleChange(0),
    idleChangeInterval(5000),          // Default interval for idle expression change
    isIdleMode(false),                 // Initialize idle mode to false
    lastExpression(-1),                // Initialize with an invalid expression
    currentBitmapOpen(neutralBitmap),  // Default to neutral open bitmap
    currentBitmapBlink(squintBitmap)   // Always blink with neutral blink bitmap
{
  calculateNextBlinkInterval();  // Initialize with a random interval
}

// Initialize the library
void SquareEyes::begin() {
  display.clearDisplay();
  drawBitmap(squintBitmap);  // Start with open eyes
  display.display();
}

// Update animation
void SquareEyes::update() {
  unsigned long currentMillis = millis();

  // If blinking or idle mode is disabled, skip the blinking logic
  if (isStaticBitmap) {
    return;
  }

  // Handle blinking
  if (!isBlinking && currentMillis - lastBlinkTime >= blinkInterval) {
    isBlinking = true;
    lastBlinkTime = currentMillis;
    drawBitmap(squintBitmap);  // Show blink eyes
    display.display();
  }

  // During the blink, optionally set a new random expression in idle mode
  if (isBlinking && currentMillis - lastBlinkTime >= blinkDuration / 2) {
    if (isIdleMode && currentMillis - lastIdleChange >= idleChangeInterval) {
      lastIdleChange = currentMillis;
      setRandomExpression();  // Set a new expression during blink
    }
  }

  // Finish the blink after `blinkDuration`
  if (isBlinking && currentMillis - lastBlinkTime >= blinkDuration) {
    isBlinking = false;
    lastBlinkTime = currentMillis;  // Reset blink timer
    calculateNextBlinkInterval();   // Set the next random interval
    drawBitmap(currentBitmapOpen);  // Return to expression eyes
    display.display();
  }
}

// Set the blink interval range
void SquareEyes::setBlinkIntervalRange(unsigned long minInterval, unsigned long maxInterval) {
  minBlinkInterval = minInterval;
  maxBlinkInterval = maxInterval;
  calculateNextBlinkInterval();
}

// Calculate the next random blink interval
void SquareEyes::calculateNextBlinkInterval() {
  blinkInterval = random(minBlinkInterval, maxBlinkInterval);
}

// Set the blink duration
void SquareEyes::setBlinkDuration(unsigned long duration) {
  blinkDuration = duration;
}

void SquareEyes::flipVertical(bool enable) {
  if (enable) {
    display.setRotation(0);  // Rotate the display 180 degrees
  } else {
    display.setRotation(2);  // Reset to default orientation
  }
  display.display();  // Apply the changes
}

// Set the expression to neutral
void SquareEyes::neutral() {
  isIdleMode = false;      // Exit idle mode
  isStaticBitmap = false;  // Enable blinking
  currentBitmapOpen = neutralBitmap;
  drawBitmap(currentBitmapOpen);  // Refresh display
  display.display();
}

// Set the expression to look left
void SquareEyes::lookLeft() {
  isIdleMode = false;      // Exit idle mode
  isStaticBitmap = false;  // Enable blinking
  currentBitmapOpen = lookLeftBitmap;
  drawBitmap(currentBitmapOpen);  // Refresh display
  display.display();
}

// Set the expression to look right
void SquareEyes::lookRight() {
  isIdleMode = false;      // Exit idle mode
  isStaticBitmap = false;  // Enable blinking
  currentBitmapOpen = lookRightBitmap;
  drawBitmap(currentBitmapOpen);  // Refresh display
  display.display();
}

// Set the expression to wonder
void SquareEyes::wonder() {
  isIdleMode = false;      // Exit idle mode
  isStaticBitmap = false;  // Enable blinking
  currentBitmapOpen = wonderingBitmap;
  drawBitmap(currentBitmapOpen);  // Refresh display
  display.display();
}

// Set the expression to annoyed
void SquareEyes::annoyed() {
  isIdleMode = false;      // Exit idle mode
  isStaticBitmap = false;  // Enable blinking
  currentBitmapOpen = annoyedBitmap;
  drawBitmap(currentBitmapOpen);  // Refresh display
  display.display();
}

// Set the expression to cute
void SquareEyes::dizzy() {
  isIdleMode = false;      // Exit idle mode
  isStaticBitmap = false;  // Enable blinking
  currentBitmapOpen = dizzyBitmap;
  drawBitmap(currentBitmapOpen);  // Refresh display
  display.display();
}

void SquareEyes::charging() {
  isIdleMode = false;                  // Disable idle mode
  isStaticBitmap = true;               // Enable blinking
  currentBitmapOpen = chargingBitmap;  // Set the custom bitmap
  drawBitmap(currentBitmapOpen);       // Display the custom bitmap
  display.display();
}

void SquareEyes::chargingComplete() {
  isIdleMode = false;                          // Disable idle mode
  isStaticBitmap = true;                       // Enable blinking
  currentBitmapOpen = chargingCompleteBitmap;  // Set the custom bitmap
  drawBitmap(currentBitmapOpen);               // Display the custom bitmap
  display.display();
}

void SquareEyes::cleanMode() {
  isIdleMode = false;                   // Disable idle mode
  isStaticBitmap = true;                // Enable blinking
  currentBitmapOpen = cleanModeBitmap;  // Set the custom bitmap
  drawBitmap(currentBitmapOpen);        // Display the custom bitmap
  display.display();
}

// Idle mode: Randomly change expressions
void SquareEyes::idle() {
  isIdleMode = true;          // Enable idle mode
  isStaticBitmap = false;     // Enable blinking
  idleChangeInterval = 3200;  // Default idle change interval
}

// Set a random expression
void SquareEyes::setRandomExpression() {
  if (!isIdleMode) return;  // Do nothing if not in idle mode

  int newExpression;

  do {
    newExpression = random(0, 6);             // Generate a random expression index (0 to 3)
  } while (newExpression == lastExpression);  // Ensure it's different from the last expression

  lastExpression = newExpression;  // Update the last expression

  // Set the new expression and refresh display
  switch (newExpression) {
    case 0:
      currentBitmapOpen = neutralBitmap;
      break;
    case 1:
      currentBitmapOpen = lookLeftBitmap;
      break;
    case 2:
      currentBitmapOpen = lookRightBitmap;
      break;
    case 3:
      currentBitmapOpen = wonderingBitmap;
      break;
    case 4:
      currentBitmapOpen = annoyedBitmap;
      break;
  }

  // Serial.print("New Expression: ");
  // Serial.println(newExpression);

  drawBitmap(currentBitmapOpen);  // Refresh the display
  display.display();
}

// Draw a single bitmap
void SquareEyes::drawBitmap(const uint8_t *bitmap) {
  display.clearDisplay();
  display.drawBitmap((display.width() - bitmapWidth) / 2,    // Center horizontally
                     (display.height() - bitmapHeight) / 2,  // Center vertically
                     bitmap, bitmapWidth, bitmapHeight, SSD1306_WHITE);
}