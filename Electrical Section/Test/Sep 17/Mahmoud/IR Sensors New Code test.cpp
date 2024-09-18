#include <Arduino.h>

/* ------------------------- Define Ultrasonics Pins ------------------------ */
#pragma region Define Ultrasonics Pins

// Front Left Sensor.
#define TRIG_FRONT_LEFT 13
#define ECHO_FRONT_LEFT 12

// Front Right Sensor.
#define TRIG_FRONT_RIGHT 11
#define ECHO_FRONT_RIGHT 19

// Right Sensor.
#define TRIG_RIGHT 10
#define ECHO_RIGHT 25

// Left Sensor.
#define TRIG_LEFT 33
#define ECHO_LEFT 14

// Back Sensor.
#define TRIG_BACK 36
#define ECHO_BACK 11

#pragma endregion

/* -------------------------- Define IR Sensor Pins ------------------------- */
#pragma region Define IR Sensor Pins

#define IR_FRONT_LEFT_PIN 5
#define IR_FRONT_RIGHT_PIN 19
#define IR_BACK_LEFT_PIN 35
#define IR_BACK_RIGHT_PIN 25

#pragma endregion

/* ---------------------------- Define Motors Pi ---------------------------- */
#pragma region Define Motors Pi

#define LEFT_MOTOR_P1 23
#define LEFT_MOTOR_P2 22
#define RIGHT_MOTOR_P1 19
#define RIGHT_MOTOR_P2 21

#pragma endregion

/* ---------------------------- Define Constants ---------------------------- */
#pragma region Define Constants

#define SPEED 255
#define minDistance 140
#define TURN_SPEED 150

#pragma endregion

/* ------------------------------- Magnets Pins ------------------------------ */
#pragma region Magnets Pins

#define MAGNET1_PIN 1
#define MAGNET2_PIN 2
#define MAGNET3_PIN 3
#define MAGNET_COUNT 3

#pragma endregion

/* ------------------------ Class Forward Declaration ----------------------- */
class MagnetsController;

/* -------------------------- Controller Instances -------------------------- */
MagnetsController controllerMagnets(MAGNET1_PIN, MAGNET2_PIN, MAGNET3_PIN);
IRController controllerIR(IR_FRONT_LEFT_PIN, IR_FRONT_RIGHT_PIN, IR_BACK_RIGHT_PIN, IR_BACK_LEFT_PIN);

/* ---------------------------- Ultrasonic Class ---------------------------- */
class Ultrasonic
{
private:
    int _trigPin;
    int _echoPin;

public:
    Ultrasonic(int trigPin, int echoPin)
    {
        _trigPin = trigPin;
        _echoPin = echoPin;

        pinMode(_trigPin, OUTPUT);
        pinMode(_echoPin, INPUT);
    }
    float read()
    {
        digitalWrite(_trigPin, LOW);
        delayMicroseconds(2);

        digitalWrite(_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(_trigPin, LOW);

        long duration = pulseIn(_echoPin, HIGH);

        float distance = duration * 0.034 / 2;

        return distance;
    }
};

/* ------------------------------- Motor Class ------------------------------ */
class Motor
{
private:
    int _leftMotorPin1;
    int _leftMotorPin2;
    int _rightMotorPin1;
    int _rightMotorPin2;

public:
    Motor(int leftMotorPin1, int leftMotorPin2, int rightMotorPin1, int rightMotorPin2)
    {
        _leftMotorPin1 = leftMotorPin1;
        _leftMotorPin2 = leftMotorPin2;
        _rightMotorPin1 = rightMotorPin1;
        _rightMotorPin2 = rightMotorPin2;

        pinMode(_leftMotorPin1, OUTPUT);
        pinMode(_leftMotorPin2, OUTPUT);
        pinMode(_rightMotorPin1, OUTPUT);
        pinMode(_rightMotorPin2, OUTPUT);
    }

    void moveForward(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
    }

    void moveBackward(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
    }

    void turnLeft(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
    }

    void turnRight(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
    }

    void stop()
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, 0);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, 0);
    }

    void setMotorSpeed(int motorPin1, int motorPin2, int speed)
    {
        if (speed > 0)
        {
            analogWrite(motorPin1, speed);
            analogWrite(motorPin2, 0);
        }
        else if (speed < 0)
        {
            analogWrite(motorPin1, 0);
            analogWrite(motorPin2, -speed);
        }
        else
        {
            analogWrite(motorPin1, 0);
            analogWrite(motorPin2, 0);
        }
    }
};

/* --------------------------- IR Sensor Position --------------------------- */
enum IRPos
{
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    BACK_RIGHT,
    BACK_LEFT,
    SENSOR_COUNT
};

/* -------------------------------- IR States ------------------------------- */
enum IRStates
{
    FRONT_LEFT = 0b1000,
    FRONT_RIGHT = 0b0100,
    BACK_RIGHT = 0b0010,
    BACK_LEFT = 0b0001,
};

/* ----------------------------- IR Sensor Class ---------------------------- */
class IRSensor
{
private:
    int _irPin;

public:
    IRSensor() {}

    IRSensor(int irPin)
    {
        _irPin = irPin;
        pinMode(_irPin, INPUT);
    }

    bool detectWhiteLine()
    {
        return digitalRead(_irPin) == LOW; // Assuming LOW means white line detected
    }
};

/* ------------------------------ IR Controller ----------------------------- */
class IRController
{
private:
    IRSensor sensors[IRPos::SENSOR_COUNT];

public:
    IRController(int FRONT_LEFT_PIN, int FRONT_RIGHT_PIN, int BACK_RIGHT_PIN, int BACK_LEFT_PIN)
    {
        sensors[IRPos::FRONT_LEFT] = IRSensor(FRONT_LEFT_PIN);
        sensors[IRPos::FRONT_RIGHT] = IRSensor(FRONT_RIGHT_PIN);
        sensors[IRPos::BACK_RIGHT] = IRSensor(BACK_RIGHT_PIN);
        sensors[IRPos::BACK_LEFT] = IRSensor(BACK_LEFT_PIN);
    }

    char checkIRSensors()
    {
        char result = 0x00;

        for (int i = 0; i < IRPos::SENSOR_COUNT; i++)
            result |= (sensors[i].detectWhiteLine() << (IRPos::SENSOR_COUNT - 1 - i));

        return result;
    }

    bool handleIRSensors(IRStates state)
    {

        switch (state)
        {
        /* ------------------------------- One Sensor ------------------------------- */
        case IRStates::FRONT_LEFT:
            Serial.println("FRONT_LEFT");
            break;
        case IRStates::FRONT_RIGHT:
            Serial.println("FRONT_RIGHT");
            break;
        case IRStates::BACK_LEFT:
            Serial.println("BACK_LEFT");
            break;
        case IRStates::BACK_RIGHT:
            Serial.println("BACK_RIGHT");
            break;

        /* ------------------------------- Two Sensors ------------------------------ */
        // Left Side..
        case (IRStates::FRONT_LEFT | IRStates::BACK_LEFT):
            Serial.println("FRONT_LEFT and BACK_LEFT");
            break;
        // Front Side..
        case (IRStates::FRONT_LEFT | IRStates::FRONT_RIGHT):
            Serial.println("FRONT_LEFT and FRONT_RIGHT");
            break;
        // Right Side..
        case (IRStates::FRONT_RIGHT | IRStates::BACK_RIGHT):
            Serial.println("FRONT_RIGHT and BACK_RIGHT");
            break;
        // Back Side..
        case (IRStates::BACK_LEFT | IRStates::BACK_RIGHT):
            Serial.println("BACK_LEFT and BACK_RIGHT");
            break;

        default:
            Serial.println("None");
            return false;
        }

        switch (state)
        {
        /* ------------------------------- One Sensor ------------------------------- */
        case IRStates::FRONT_LEFT:
            controllerMagnets.turnOn();
            break;
        case IRStates::FRONT_RIGHT:
            controllerMagnets.turnOn();
            break;
        case IRStates::BACK_LEFT:

            break;
        case IRStates::BACK_RIGHT:
            Serial.println("BACK_RIGHT");
            break;

        /* ------------------------------- Two Sensors ------------------------------ */
        // Left Side..
        case (IRStates::FRONT_LEFT | IRStates::BACK_LEFT):
            Serial.println("FRONT_LEFT and BACK_LEFT");
            break;
        // Front Side..
        case (IRStates::FRONT_LEFT | IRStates::FRONT_RIGHT):
            Serial.println("FRONT_LEFT and FRONT_RIGHT");
            break;
        // Right Side..
        case (IRStates::FRONT_RIGHT | IRStates::BACK_RIGHT):
            Serial.println("FRONT_RIGHT and BACK_RIGHT");
            break;
        // Back Side..
        case (IRStates::BACK_LEFT | IRStates::BACK_RIGHT):
            Serial.println("BACK_LEFT and BACK_RIGHT");
            break;

        default:
            Serial.println("None");
            return false;
        }

        return true;
    }
};

// /* -------------------------------- IR States ------------------------------- */
// enum IRStates
// {
//     _NONE,
//     _BL,
//     _BR,
//     _BLBR,
//     _FR,
//     _FRBL,
//     _FRBR,
//     _FRBRBL,
//     _FL,
//     _FLBL,
//     _FLBR,
//     _FLBRBL,
//     _FLFR,
//     _FLFRBL,
//     _FLFRBR,
//     _FLFRBRBL,
// };

/* ----------------------- Ultrasonic Sensor Positions ---------------------- */
enum SensorPosition_t
{
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK,
    LEFT,
    RIGHT
};

/* --------------------------- Magnets Controller --------------------------- */
class MagnetsController
{
private:
    int magnetPins[MAGNET_COUNT];

public:
    MagnetsController(int magnet1Pin, int magnet2Pin, int magnet3Pin)
    {
        // Assign pins.
        this->magnetPins[0] = magnet1Pin;
        this->magnetPins[1] = magnet2Pin;
        this->magnetPins[2] = magnet3Pin;

        // Initialize pin mode.
        for (int i = 0; i < MAGNET_COUNT; i++)
            pinMode(magnetPins[i], OUTPUT);
    }

    /// @brief Turns on all magnets.
    void turnOn()
    {
        for (int i = 0; i < MAGNET_COUNT; i++)
            digitalWrite(magnetPins[i], HIGH);
    }

    /// @brief Turns off all magnets.
    void turnOff()
    {
        for (int i = 0; i < MAGNET_COUNT; i++)
            digitalWrite(magnetPins[i], LOW);
    }
};

/* ------------------------ Ultrasonic Sensor Objects ----------------------- */
Ultrasonic frontLeft(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
Ultrasonic frontRight(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
Ultrasonic back(TRIG_BACK, ECHO_BACK);
Ultrasonic left(TRIG_LEFT, ECHO_LEFT);
Ultrasonic right(TRIG_RIGHT, ECHO_RIGHT);

/* ---------------------------- IR Sensor Objects --------------------------- */

/* ------------------------------ Motor Object ------------------------------ */
Motor motor(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

/* -------------------------- Function Declaration -------------------------- */
float measureDistance(SensorPosition_t SensorPosition);
char checkIRSensors();
void handleIRSensors(IRStates state);
void handleUltrasonic();
void turn(int angle);

void setup()
{
    pinMode(2, OUTPUT);

    Serial.begin(115200);
}

void loop()
{
    char result = checkIRSensors();
    handleIRSensors((IRStates)result);
    delay(250);
}

void handleUltrasonic()
{
    float frontLeftDistance = measureDistance(FRONT_LEFT);
    delay(10);
    float frontRightDistance = measureDistance(FRONT_RIGHT);
    delay(10);
    float backDistance = measureDistance(BACK);
    delay(10);
    float leftDistance = measureDistance(LEFT);
    delay(10);
    float rightDistance = measureDistance(RIGHT);
    delay(10);

    if (frontLeftDistance < minDistance && frontRightDistance < minDistance)
    {
        motor.moveForward(SPEED);
        delay(10);
    }
    else if (leftDistance < minDistance || (frontLeftDistance < minDistance && frontRightDistance > minDistance))
    {
        while (frontLeftDistance > minDistance || frontRightDistance > minDistance)
        {
            motor.turnLeft(TURN_SPEED);
            delay(10);
            frontLeftDistance = measureDistance(FRONT_LEFT);
            frontRightDistance = measureDistance(FRONT_RIGHT);
        }
    }
    else if (backDistance < minDistance || rightDistance < minDistance || (frontLeftDistance > minDistance && frontRightDistance < minDistance))
    {
        while (frontLeftDistance > minDistance || frontRightDistance > minDistance)
        {
            motor.turnRight(TURN_SPEED);
            delay(10);
            frontLeftDistance = measureDistance(FRONT_LEFT);
            frontRightDistance = measureDistance(FRONT_RIGHT);
        }
    }
    else
    {
        while (frontLeftDistance > minDistance || frontRightDistance > minDistance)
        {
            motor.turnRight(TURN_SPEED);
            delay(10);
            frontLeftDistance = measureDistance(FRONT_LEFT);
            frontRightDistance = measureDistance(FRONT_RIGHT);
        }
    }
}

float measureDistance(SensorPosition_t SensorPosition)
{
    switch (SensorPosition)
    {
    case FRONT_LEFT:
        return frontLeft.read();
    case FRONT_RIGHT:
        return frontRight.read();
    case BACK:
        return back.read();
    case LEFT:
        return left.read();
    case RIGHT:
        return right.read();
    default:
        return 0;
    }
}

void turn(int angle)
{
    // mpu.updateReadings();
    // float current_angle = gyroAngle();
    // float target_angle = current_angle + angle;

    // while (current_angle != target_angle)
    // {
    //     mpu.updateReadings();
    //     current_angle = gyroAngle();
    //     if (angle > 0)
    //     {
    //         turnRight();
    //     }
    //     else
    //     {
    //         turnLeft();
    //     }
    // }

    // motors.stop();
}
