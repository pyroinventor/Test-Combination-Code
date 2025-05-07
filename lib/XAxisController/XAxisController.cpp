#include "XAxisController.h"

// Constructor implementation
XAxisController::XAxisController(
    int stepPin, int stepPinN,
    int dirPin, int dirPinN,
    int enPin, int enPinN,
    int limitSw) : STEP_PIN(stepPin), STEP_PIN_N(stepPinN),
                   DIR_PIN(dirPin), DIR_PIN_N(dirPinN),
                   EN_PIN(enPin), EN_PIN_N(enPinN),
                   LIMIT_SW(limitSw)
{
}

void XAxisController::begin(const char *btName)
{
    Serial.begin(115200);
    SerialBT.begin(btName);

    // Configure pins
    pinMode(STEP_PIN, OUTPUT);
    pinMode(STEP_PIN_N, OUTPUT);
    pinMode(DIR_PIN, OUTPUT_OPEN_DRAIN);
    pinMode(DIR_PIN_N, OUTPUT_OPEN_DRAIN);
    pinMode(EN_PIN, OUTPUT);
    pinMode(EN_PIN_N, OUTPUT);
    pinMode(LIMIT_SW, INPUT_PULLUP);

    // Initialize pin states
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(DIR_PIN_N, HIGH);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(STEP_PIN_N, HIGH);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(DIR_PIN_N, HIGH);
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(EN_PIN_N, LOW);

    sendResponse("X-Axis Controller Ready");
}

void XAxisController::step(int delayUs)
{
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN_N, LOW);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(STEP_PIN_N, HIGH);
    delayMicroseconds(delayUs);
}

void XAxisController::setDirection(bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(DIR_PIN, HIGH);
        digitalWrite(DIR_PIN_N, LOW);
    }
    else
    {
        digitalWrite(DIR_PIN, LOW);
        digitalWrite(DIR_PIN_N, HIGH);
    }
    delayMicroseconds(10);
}

bool XAxisController::isLimitTriggered()
{
    static bool lastState = false;
    static unsigned long lastDebounceTime = 0;
    bool currentState = digitalRead(LIMIT_SW) == LOW;

    if (currentState != lastState)
    {
        lastDebounceTime = millis();
        lastState = currentState;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
    {
        return currentState;
    }
    return false;
}

void XAxisController::sendResponse(const String &message)
{
    Serial.println(message);
    SerialBT.println(message);
}

float XAxisController::cubicBezier(float t, float p0, float p1, float p2, float p3)
{
    float oneMinusT = 1.0f - t;
    float oneMinusT2 = oneMinusT * oneMinusT;
    float t2 = t * t;
    return oneMinusT2 * oneMinusT * p0 +
           3.0f * oneMinusT2 * t * p1 +
           3.0f * oneMinusT * t2 * p2 +
           t2 * t * p3;
}

int XAxisController::calculateStepDelay(int currentStep, int totalSteps)
{
    const float p0 = 0.0f;
    const float p1 = 0.65f;
    const float p2 = 0.85f;
    const float p3 = 1.0f;

    if (currentStep < ACCEL_STEPS)
    {
        float t = (float)currentStep / ACCEL_STEPS;
        float factor = cubicBezier(t, p0, p1, p2, p3);
        return MAX_STEP_DELAY - (factor * (MAX_STEP_DELAY - MIN_STEP_DELAY));
    }
    else if (currentStep > (totalSteps - ACCEL_STEPS))
    {
        float t = (float)(totalSteps - currentStep) / ACCEL_STEPS;
        float factor = cubicBezier(t, p0, p1, p2, p3);
        return MAX_STEP_DELAY - (factor * (MAX_STEP_DELAY - MIN_STEP_DELAY));
    }
    return MIN_STEP_DELAY;
}

void XAxisController::rotateWithAccel(bool clockwise, int steps)
{
    sendResponse("Direction: " + String(clockwise ? "CLOCKWISE" : "COUNTER_CLOCKWISE"));

    if (isHomed)
    {
        long targetPosition = currentPosition + (clockwise ? steps : -steps);
        if (targetPosition > MAX_POSITION || targetPosition < MIN_POSITION)
        {
            sendResponse("Error: Movement would exceed limits");
            return;
        }
    }

    setDirection(clockwise);
    delayMicroseconds(50);

    for (int i = 0; i < steps; i++)
    {
        if (isLimitTriggered())
        {
            sendResponse("Limit switch triggered!");
            isHomed = false;
            return;
        }

        int stepDelay = calculateStepDelay(i, steps);
        step(stepDelay);

        if (isHomed)
        {
            currentPosition += (clockwise ? 1 : -1);
        }
    }

    if (isHomed)
    {
        sendResponse("Move complete - Position: " + String(currentPosition) +
                     " steps (" + String(currentPosition / STEPS_PER_MM) + " mm)");
    }
}

void XAxisController::homeAxis()
{
    sendResponse("Homing axis...");

    // Move towards limit switch
    setDirection(false);
    while (!isLimitTriggered())
    {
        step(HOMING_SPEED);
    }

    // Back off from limit
    setDirection(true);
    for (int i = 0; i < HOME_BACKOFF; i++)
    {
        step(HOMING_SPEED);
    }

    currentPosition = 0;
    isHomed = true;
    sendResponse("Homing complete");
}

void XAxisController::moveToPosition(float targetMm)
{
    if (!isHomed)
    {
        sendResponse("Error: Please home axis first");
        return;
    }

    long targetSteps = (long)(targetMm * STEPS_PER_MM);
    if (targetSteps > MAX_POSITION || targetSteps < MIN_POSITION)
    {
        sendResponse("Error: Target position out of range");
        return;
    }

    long stepsToMove = targetSteps - currentPosition;
    if (stepsToMove == 0)
    {
        sendResponse("Already at target position");
        return;
    }

    bool direction = stepsToMove > 0;
    rotateWithAccel(direction, abs(stepsToMove));
}

void XAxisController::processSerial()
{
    static String buffer = "";

    while (Serial.available() || SerialBT.available())
    {
        char c;
        if (Serial.available())
        {
            c = Serial.read();
        }
        else
        {
            c = SerialBT.read();
        }

        if (c == '\n' || c == '\r')
        {
            if (buffer.length() > 0)
            {
                processCommand(buffer);
                buffer = "";
            }
        }
        else
        {
            buffer += c;
        }
    }
}

void XAxisController::processCommand(String command)
{
    command.trim();
    char cmd = toupper(command[0]);
    String param = command.substring(1);

    switch (cmd)
    {
    case 'E':
        digitalWrite(EN_PIN, LOW);
        sendResponse("Motor enabled");
        break;

    case 'D':
        digitalWrite(EN_PIN, HIGH);
        sendResponse("Motor disabled");
        break;

    case 'H':
        homeAxis();
        break;

    case 'P':
        sendResponse("Position: " + String(currentPosition / STEPS_PER_MM) +
                     "mm, Homed: " + String(isHomed ? "Yes" : "No"));
        break;

    case 'G':
        if (param.length() > 0)
        {
            float targetPos = param.toFloat();
            moveToPosition(targetPos);
        }
        break;

    default:
        sendResponse("Unknown command");
        break;
    }
}