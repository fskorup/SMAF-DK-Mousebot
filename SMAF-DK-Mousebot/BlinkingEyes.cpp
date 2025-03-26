#include "Adafruit_SSD1306.h"
#include "BlinkingEyes.h"

BlinkingEyes::BlinkingEyes(Adafruit_SSD1306 &disp)
  : display(disp), isBlinking(false), isAnimating(false), isNormalizing(false), isDelaying(false) {

  leftEye = { 16, 30, 96, 4 };
  rightEye = { 16, 30, 96, 4 };
}

// Initialize the library
void BlinkingEyes::begin() {
  display.clearDisplay();
  display.fillRect(16, 30, 96, 4, SSD1306_WHITE);  // Filled part
  display.display();
}

// Set a new eye expression with animation
void BlinkingEyes::setExpression(ExpressionState expression) {
  isBlinking = true;
  isAnimating = false;
  isNormalizing = false;
  isDelaying = false;

  animationStartTime = millis();

  startLeft = leftEye;
  startRight = rightEye;
  blink = { 16, 30, 96, 4 };

  // Define target eye positions based on expression
  switch (expression) {
    case NEUTRAL:
      targetLeft = {
        36,  // x
        18,  // y
        20,  // w
        28   // h
      };

      targetRight = {
        72,  // x
        18,  // y
        20,  // w
        28   // h
      };
      break;
    case LOOK_LEFT:
      targetLeft = {
        18,  // x
        18,  // y
        20,  // w
        28   // h
      };

      targetRight = {
        44,  // x
        22,  // y
        20,  // w
        20   // h
      };
      break;
    case LOOK_RIGHT:
      targetLeft = {
        64,  // x
        22,  // y
        20,  // w
        20   // h
      };

      targetRight = {
        90,  // x
        18,  // y
        20,  // w
        28   // h
      };
      break;
    case ANNOYED:
      targetLeft = {
        28,  // x
        28,  // y
        28,  // w
        8    // h
      };

      targetRight = {
        72,  // x
        28,  // y
        28,  // w
        8    // h
      };
      break;
    case CURIOUS:
      targetLeft = {
        32,  // x
        26,  // y
        28,  // w
        12   // h
      };

      targetRight = {
        68,  // x
        30,  // y
        28,  // w
        8    // h
      };
      break;
  }
}

void BlinkingEyes::setChargingMode(int percentage) {
  isBlinking = false;
  isAnimating = false;
  isNormalizing = false;
  isDelaying = false;

  static int currentBarWidth = 0;  // Tracks the current progress bar width
  static unsigned long lastUpdateTime = 0;  // Time tracking for smooth animation
  const int updateInterval = 160;  // Speed of the filling effect (adjust as needed)

  percentage = constrain(percentage, 0, 100);
  int targetBarWidth = map(100, 0, 100, 0, 42);  // Always animating to 100%
  int startBarWidth = map(percentage, 0, 100, 0, 42);  // Convert input percentage to pixels

  if (currentBarWidth == 0) {
    currentBarWidth = startBarWidth;  // Initialize bar at the given percentage
  }

  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;

    // Smoothly increase the bar width until it reaches 100%
    if (currentBarWidth < targetBarWidth) {
      currentBarWidth += 1;  // Increase step by step
    } else {
      currentBarWidth = startBarWidth;  // Reset when full, restarting from given percentage
    }
  }

  // Draw battery outline
  display.clearDisplay();
  display.fillRect(38, 18, 52, 28, SSD1306_WHITE); // Battery outline
  display.fillRect(41, 21, 46, 22, SSD1306_BLACK); // Inner black background
  display.fillRect(87, 26, 6, 12, SSD1306_WHITE);  // Battery tip (connector)

  // Draw animated progress bar
  display.fillRect(43, 23, currentBarWidth, 18, SSD1306_WHITE);

  display.display();  // Refresh OLED
}

// Set a new eye expression with animation
void BlinkingEyes::setBatteryCheckMode(int percentage) {
  isBlinking = false;
  isAnimating = false;
  isNormalizing = false;
  isDelaying = false;

  animationStartTime = millis();

  display.clearDisplay();

  display.fillRect(38, 18, 52, 28, SSD1306_WHITE);
  display.fillRect(41, 21, 46, 22, SSD1306_BLACK);
  display.fillRect(87, 26, 6, 12, SSD1306_WHITE);

  // Constrain percentage between 0 and 100
  percentage = constrain(percentage, 0, 100);

  // Map percentage (0-100%) to progress bar width (0-88px)
  int barWidth = map(percentage, 0, 100, 0, 42);

  // Draw the progress bar at (20, 38)
  display.fillRect(43, 23, barWidth, 18, SSD1306_WHITE);  // Filled part
  display.display();  // Refresh display
}

// Set a new eye expression with animation
void BlinkingEyes::setLowBatteryWarning() {
  isBlinking = false;
  isAnimating = false;
  isNormalizing = false;
  isDelaying = false;

  animationStartTime = millis();

  display.clearDisplay();
  display.fillRect(38, 18, 52, 28, SSD1306_WHITE);
  display.fillRect(41, 21, 46, 22, SSD1306_BLACK);
  display.fillRect(87, 26, 6, 12, SSD1306_WHITE);
  display.fillRect(62, 24, 4, 11, SSD1306_WHITE);
  display.fillRect(62, 37, 4, 3, SSD1306_WHITE);

  display.display();  // Refresh display
}

// Set a new eye expression with animation
void BlinkingEyes::setChargingCompleteMode() {
  isBlinking = false;
  isAnimating = false;
  isNormalizing = false;
  isDelaying = false;

  animationStartTime = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Charging Complete");
  display.display();  // Refresh display
}

// Set a new eye expression with animation
void BlinkingEyes::setWheelCleanMode() {
  isBlinking = false;
  isAnimating = false;
  isNormalizing = false;
  isDelaying = false;

  animationStartTime = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Cleaning wheels");
  display.display();  // Refresh display
}

void BlinkingEyes::update() {
  unsigned long currentTime = millis();

  if (isBlinking) {
    float t = (float)(currentTime - animationStartTime) / 80;  // Fast blink animation
    if (t > 1) {
      t = 1;
      isBlinking = false;
      isNormalizing = true;
      animationStartTime = millis();  // Reset animation timer for next phase
    }

    // Interpolate eye positions smoothly to blink
    leftEye.x = startLeft.x + t * (blink.x - startLeft.x);
    leftEye.y = startLeft.y + t * (blink.y - startLeft.y);
    leftEye.width = startLeft.width + t * (blink.width - startLeft.width);
    leftEye.height = startLeft.height + t * (blink.height - startLeft.height);

    rightEye.x = startRight.x + t * (blink.x - startRight.x);
    rightEye.y = startRight.y + t * (blink.y - startRight.y);
    rightEye.width = startRight.width + t * (blink.width - startRight.width);
    rightEye.height = startRight.height + t * (blink.height - startRight.height);
  }

  if (isNormalizing) {
    float t = (float)(currentTime - animationStartTime) / 80;  // Fast normalization
    if (t > 1) {
      t = 1;
      isNormalizing = false;
      isDelaying = true;
      delayStartTime = millis();  // Start delay timer (new variable)
    }

    // Fix math: Interpolate from blink back to normal
    leftEye.x = blink.x + t * (startLeft.x - blink.x);
    leftEye.y = blink.y + t * (startLeft.y - blink.y);
    leftEye.width = blink.width + t * (startLeft.width - blink.width);
    leftEye.height = blink.height + t * (startLeft.height - blink.height);

    rightEye.x = blink.x + t * (startRight.x - blink.x);
    rightEye.y = blink.y + t * (startRight.y - blink.y);
    rightEye.width = blink.width + t * (startRight.width - blink.width);
    rightEye.height = blink.height + t * (startRight.height - blink.height);
  }

  if (isDelaying) {
    if (currentTime - delayStartTime >= 240) {
      isDelaying = false;
      isAnimating = true;
      animationStartTime = millis();  // Reset timer for final animation
    }
  }

  // Handle animation to new expression
  if (isAnimating) {
    float t = (float)(currentTime - animationStartTime) / 160;
    if (t > 1) {
      t = 1;
      isAnimating = false;  // Animation finished
    }

    // Interpolate eye positions smoothly to target
    leftEye.x = startLeft.x + t * (targetLeft.x - startLeft.x);
    leftEye.y = startLeft.y + t * (targetLeft.y - startLeft.y);
    leftEye.width = startLeft.width + t * (targetLeft.width - startLeft.width);
    leftEye.height = startLeft.height + t * (targetLeft.height - startLeft.height);

    rightEye.x = startRight.x + t * (targetRight.x - startRight.x);
    rightEye.y = startRight.y + t * (targetRight.y - startRight.y);
    rightEye.width = startRight.width + t * (targetRight.width - startRight.width);
    rightEye.height = startRight.height + t * (targetRight.height - startRight.height);
  }

  // Drawing logic
  display.clearDisplay();
  drawEyes();
  display.display();
}

void BlinkingEyes::drawEyes() {
  display.fillRect(leftEye.x, leftEye.y, leftEye.width, leftEye.height, WHITE);
  display.fillRect(rightEye.x, rightEye.y, rightEye.width, rightEye.height, WHITE);
}