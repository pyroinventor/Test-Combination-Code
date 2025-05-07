#include "MotorController.h"

MotorController::MotorController(
    int pwmPin, int enablePin, int dirPin,
    int ledPin, int speedFbPin, int servoFbPin) : PWM_OUT_PIN(pwmPin),
                                                  ENABLE_PIN(enablePin),
                                                  DIR_PIN(dirPin),
                                                  LED_PIN(ledPin),
                                                  SPEED_FEEDBACK_PIN(speedFbPin),
                                                  SERVO_FEEDBACK_PIN(servoFbPin),
                                                  PWM_CHANNEL(0),
                                                  PWM_FREQ(20000),
                                                  PWM_RESOLUTION(8)
{
}

void MotorController::begin()
{
    // Configure pins
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SPEED_FEEDBACK_PIN, INPUT);
    pinMode(SERVO_FEEDBACK_PIN, INPUT);

    // Configure PWM
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_OUT_PIN, PWM_CHANNEL);

    // Initialize state
    enable(false);
    setDirection(true);
    setSpeed(0);
    calibrateSpeedOffset();
}

void MotorController::enable(bool state)
{
    motorEnabled = state;
    digitalWrite(ENABLE_PIN, state);
    digitalWrite(LED_PIN, state);
}

void MotorController::setDirection(bool forward)
{
    motorDirection = forward;
    digitalWrite(DIR_PIN, forward);
}

void MotorController::setSpeed(float percent)
{
    currentPercent = constrain(percent, 0.0, 100.0);
    float pwmValue = map(currentPercent, 0, 100, 0, 255);
    ledcWrite(PWM_CHANNEL, (int)pwmValue);
}

float MotorController::readSpeedFeedback()
{
    int rawValue = analogRead(SPEED_FEEDBACK_PIN);
    return (rawValue / 4095.0) * 3.3;
}

float MotorController::calculateRPM()
{
    static const float VOLTS_TO_RPM = 1000.0; // Adjust based on your motor
    float voltage = readSpeedFeedback() - speedOffset;
    return voltage * VOLTS_TO_RPM;
}

void MotorController::calibrateSpeedOffset()
{
    const int samples = 10;
    float sum = 0;

    for (int i = 0; i < samples; i++)
    {
        sum += readSpeedFeedback();
        delay(10);
    }

    speedOffset = sum / samples;
}

float MotorController::smoothRPM(float newRPM)
{
    static const float alpha = 0.2;
    static float smoothedRPM = 0;

    smoothedRPM = (alpha * newRPM) + ((1.0 - alpha) * smoothedRPM);
    return smoothedRPM;
}

void MotorController::displayStatus()
{
    unsigned long currentMillis = millis();
    if (currentMillis - lastDisplayTime >= 1000)
    {
        lastDisplayTime = currentMillis;

        Serial.print("Speed: ");
        Serial.print(currentPercent);
        Serial.print("% | RPM: ");
        Serial.print(currentRPM);
        Serial.print(" | Enabled: ");
        Serial.print(motorEnabled ? "Yes" : "No");
        Serial.print(" | Direction: ");
        Serial.println(motorDirection ? "Forward" : "Reverse");
    }
}

void MotorController::update()
{
    unsigned long currentMillis = millis();

    // Update RPM calculation every 100ms
    if (currentMillis - lastSpeedRead >= 100)
    {
        lastSpeedRead = currentMillis;
        if (motorEnabled)
        {
            float rawRPM = calculateRPM();
            currentRPM = smoothRPM(rawRPM);
        }
        else
        {
            currentRPM = 0;
        }
    }

    // Update display every second
    if (currentMillis - lastRpmDisplay >= 1000)
    {
        lastRpmDisplay = currentMillis;
        displayStatus();
    }
}

void MotorController::processSerial()
{
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        char cmd = toupper(command[0]);
        String param = command.substring(1);

        switch (cmd)
        {
        case 'S': // Set speed (0-100)
            setSpeed(param.toFloat());
            break;

        case 'E': // Enable/Disable (1/0)
            enable(param.toInt() == 1);
            break;

        case 'D': // Direction (1=forward, 0=reverse)
            setDirection(param.toInt() == 1);
            break;

        case 'P': // Print status
            displayStatus();
            break;
        }
    }
}