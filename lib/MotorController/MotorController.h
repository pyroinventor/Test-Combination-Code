#pragma once
#include <Arduino.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>

class MotorController
{
private:
    // Pin configuration
    const int PWM_OUT_PIN;
    const int ENABLE_PIN;
    const int DIR_PIN;
    const int LED_PIN;
    const int SPEED_FEEDBACK_PIN;
    const int SERVO_FEEDBACK_PIN;

    // PWM configuration
    const int PWM_CHANNEL;
    const int PWM_FREQ;
    const int PWM_RESOLUTION;

    // State variables
    float currentPercent = 0.0;
    float currentRPM = 0.0;
    float currentPWM = 0.0;
    bool motorEnabled = false;
    bool motorDirection = true;
    bool ledState = false;

    // Timing variables
    unsigned long previousMillis = 0;
    unsigned long lastSpeedRead = 0;
    unsigned long lastRpmDisplay = 0;
    unsigned long lastDisplayTime = 0;
    float speedOffset = 0.0;
    float lastPosition = 0.0;
    unsigned long lastPositionTime = 0;

    // Private methods
    float readPWMDutyCycle(int channel);
    void calibrateSpeedOffset();
    float smoothRPM(float newRPM);
    void displayFeedback();
    float calculateRPM();
    float readSpeedFeedback();
    float readDrivePWM();

public:
    MotorController(
        int pwmPin, int enablePin, int dirPin,
        int ledPin, int speedFbPin, int servoFbPin);

    void begin();
    void processSerial();
    void update();
    void setSpeed(float percent);
    void enable(bool state);
    void setDirection(bool forward);
    void displayStatus();

    // Getters
    float getCurrentSpeed() const { return currentPercent; }
    bool isEnabled() const { return motorEnabled; }
    bool getDirection() const { return motorDirection; }
};