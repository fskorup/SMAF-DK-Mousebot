#ifndef BLINKINGEYES_H
#define BLINKINGEYES_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

enum ExpressionState { NEUTRAL,
                       LOOK_LEFT,
                       LOOK_RIGHT,
                       ANNOYED,
                       CURIOUS };

struct Eye {
  int x, y, width, height;
};

class BlinkingEyes {
public:
  BlinkingEyes(Adafruit_SSD1306 &display);

  void begin();
  void update();

  void setExpression(ExpressionState expression);
  void setLowBatteryWarning();
  void setChargingMode(int percentage);
  void setBatteryCheckMode(int percentage);
  void setChargingCompleteMode();
  void setWheelCleanMode();

private:
  Adafruit_SSD1306 &display;
  Eye leftEye, rightEye;
  Eye startLeft, startRight, targetLeft, targetRight, blink;

  bool isBlinking;
  bool isNormalizing;
  bool isAnimating;
  bool isDelaying;
  unsigned long delayStartTime;  // Stores the start time for the delay phase
  bool isExpressionActive;
  int blinkX, blinkY, blinkWidth, blinkHeight, blinkDuration, blinkDelay;
  unsigned long lastBlinkTime, blinkStartTime, lastExpressionTime;
  unsigned long animationStartTime;
  int animationDuration;

  int animationStep = 0;                // Tracks the pre-animation steps
  bool isPreAnimationDone = true;       // Prevents pre-animation from running after the first time
  unsigned long preAnimationStartTime;  // Stores time for pre-animation timing

  ExpressionState currentExpression;
  bool isTransitioning;                   // Tracks whether we're in the transition from blink to expression
  unsigned long nextTargetSetTime;        // Time to switch from blink to expression
  ExpressionState nextExpression;         // Stores the next expression to switch after blink
  unsigned long holdExpressionStartTime;  // Time to keep the old expression before switching
  bool isHoldingCurrentExpression;        // Tracks whether we are holding the old expression before switching

  void drawEyes();
  void interpolateEyes();
};

#endif