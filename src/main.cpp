#include <Arduino.h>
#include "XAxisController.h"
#include "MotorController.h"

// Create controller instances
XAxisController xAxis(
    23, 22, // STEP pins
    21, 19, // DIR pins
    18, 5,  // EN pins
    4       // LIMIT pin
);

MotorController motor(
    25, // PWM_OUT_PIN
    26, // ENABLE_PIN
    27, // DIR_PIN
    2,  // LED_PIN
    32, // SPEED_FEEDBACK_PIN
    33  // SERVO_FEEDBACK_PIN
);

// Test sequence states
enum TestState
{
    INIT,
    HOMING,
    MOVE_TO_START,
    SPINDLE_START,
    MOVING,
    SPINDLE_STOP,
    COMPLETE
};

TestState currentState = INIT;
unsigned long stateStartTime = 0;

void setup()
{
    Serial.begin(115200);
    xAxis.begin("X-Axis Test");
    motor.begin();

    Serial.println("Test Program Started");
    Serial.println("Press 'S' to start test sequence");
}

void loop()
{
    xAxis.processSerial();
    motor.processSerial();
    motor.update();

    if (Serial.available())
    {
        char c = Serial.read();
        if (toupper(c) == 'S' && currentState == INIT)
        {
            currentState = HOMING;
            stateStartTime = millis();
            Serial.println("Starting test sequence...");
        }
    }

    // Test sequence state machine
    switch (currentState)
    {
    case HOMING:
        xAxis.homeAxis();
        currentState = MOVE_TO_START;
        stateStartTime = millis();
        break;

    case MOVE_TO_START:
        if (millis() - stateStartTime > 1000)
        {
            xAxis.moveToPosition(0);
            currentState = SPINDLE_START;
            stateStartTime = millis();
        }
        break;

    case SPINDLE_START:
        if (millis() - stateStartTime > 1000)
        {
            motor.enable(true);
            motor.setDirection(true);
            motor.setSpeed(50); // 50% speed
            currentState = MOVING;
            stateStartTime = millis();
        }
        break;

    case MOVING:
        if (millis() - stateStartTime > 2000)
        {
            // Move to 100mm while spindle is running
            xAxis.moveToPosition(100);
            currentState = SPINDLE_STOP;
            stateStartTime = millis();
        }
        break;

    case SPINDLE_STOP:
        if (millis() - stateStartTime > 5000)
        {
            motor.setSpeed(0);
            motor.enable(false);
            currentState = COMPLETE;
            Serial.println("Test sequence complete!");
        }
        break;

    case COMPLETE:
        // Do nothing, wait for next start command
        break;

    default:
        break;
    }
}