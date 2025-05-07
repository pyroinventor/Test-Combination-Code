#pragma once
#include <Arduino.h>
#include "BluetoothSerial.h"

class XAxisController
{
public:
    XAxisController(
        int stepPin, int stepPinN,
        int dirPin, int dirPinN,
        int enPin, int enPinN,
        int limitSw);

    void begin(const char *btName = "X-Axis Controller");
    void processSerial();
    void enable();
    void disable();
    void homeAxis();
    void moveToPosition(float targetMm);
    float getCurrentPosition() const { return currentPosition / STEPS_PER_MM; }
    bool isAxisHomed() const { return isHomed; }

private:
    // Pin configuration
    const int STEP_PIN, STEP_PIN_N;
    const int DIR_PIN, DIR_PIN_N;
    const int EN_PIN, EN_PIN_N;
    const int LIMIT_SW;

    // Constants
    static const int HOME_BACKOFF = 500;
    static const int HOMING_SPEED = 500;
    static const int TOTAL_STEPS = 2000;
    static const long MAX_POSITION = 112000;
    static const long MIN_POSITION = 0;
    static const int MIN_STEP_DELAY = 150;
    static const int MAX_STEP_DELAY = 2000;
    static const int ACCEL_STEPS = 1000;
    static const float STEPS_PER_MM = 200.0;
    static const int DEBOUNCE_DELAY = 50;

    // State variables
    long currentPosition = 0;
    bool isHomed = false;
    BluetoothSerial SerialBT;

    // Helper functions
    bool isLimitTriggered();
    void setDirection(bool clockwise);
    void step(int delayUs);
    float cubicBezier(float t, float p0, float p1, float p2, float p3);
    int calculateStepDelay(int currentStep, int totalSteps);
    void sendResponse(const String &message);
    void rotateWithAccel(bool clockwise, int steps);
    void processCommand(String command);
};