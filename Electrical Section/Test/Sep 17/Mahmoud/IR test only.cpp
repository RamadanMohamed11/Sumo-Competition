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

/* ---------------------------- Define Motors Pis ---------------------------- */
#define LEFT_MOTOR_P1 33
#define LEFT_MOTOR_P2 25
#define RIGHT_MOTOR_P1 35
#define RIGHT_MOTOR_P2 32

/* ---------------------------- Define Constants ---------------------------- */
#define SPEED 255
#define minDistance 140
#define TURN_SPEED 150

/* ------------------------------- Magnets Pins ------------------------------ */
#define MAGNET1_PIN 1
#define MAGNET2_PIN 2
#define MAGNET3_PIN 3
#define MAGNET_COUNT 3

/* -------------------------- Define IR Sensor Pins ------------------------- */
#define IR_FRONT_LEFT_PIN 4
#define IR_FRONT_RIGHT_PIN 5
#define IR_BACK_LEFT_PIN 19
#define IR_BACK_RIGHT_PIN 21

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

/* ------------------------------ Motor Object ------------------------------ */
Motor motor(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

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

/* --------------------------- IR Sensor Position --------------------------- */
enum IRPos
{
    _FL = 0,
    _FR,
    _BR,
    _BL,
    IR_SENSOR_COUNT,
};

/* -------------------------------- IR States ------------------------------- */
enum IRStates
{
    WHITE_FRONT_LEFT = 0b1000,
    WHITE_FRONT_RIGHT = 0b0100,
    WHITE_BACK_RIGHT = 0b0010,
    WHITE_BACK_LEFT = 0b0001,
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

    int getAnalog()
    {
        return analogRead(_irPin);
    }
};

/* ------------------------------ IR Controller ----------------------------- */
class IRController
{
private:
    IRSensor sensors[IRPos::IR_SENSOR_COUNT];

public:
    IRController(int FRONT_LEFT_PIN, int FRONT_RIGHT_PIN, int BACK_RIGHT_PIN, int BACK_LEFT_PIN)
    {
        sensors[IRPos::_FL] = IRSensor(FRONT_LEFT_PIN);
        sensors[IRPos::_FR] = IRSensor(FRONT_RIGHT_PIN);
        sensors[IRPos::_BR] = IRSensor(BACK_RIGHT_PIN);
        sensors[IRPos::_BL] = IRSensor(BACK_LEFT_PIN);
    }

    char checkIRSensors()
    {
        char result = 0x00;

        for (int i = 0; i < IRPos::IR_SENSOR_COUNT; i++)
            result |= (sensors[i].detectWhiteLine() << (IRPos::IR_SENSOR_COUNT - 1 - i));

        return result;
    }

    bool handleIRSensors(IRStates state)
    {

        switch (state)
        {
        /* ------------------------------- One Sensor ------------------------------- */
        case IRStates::WHITE_FRONT_LEFT:
            Serial.println("FRONT_LEFT");
            break;
        case IRStates::WHITE_FRONT_RIGHT:
            Serial.println("FRONT_RIGHT");
            break;
        case IRStates::WHITE_BACK_LEFT:
            Serial.println("BACK_LEFT");
            break;
        case IRStates::WHITE_BACK_RIGHT:
            Serial.println("BACK_RIGHT");
            break;

        /* ------------------------------- Two Sensors ------------------------------ */
        // Left Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_BACK_LEFT):
            Serial.println("FRONT_LEFT and BACK_LEFT");
            motor.turnRight(SPEED);
            delay(1000);
            motor.moveForward(SPEED);
            break;
        // Front Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
            Serial.println("FRONT_LEFT and FRONT_RIGHT");
            motor.moveBackward(SPEED);
            break;
        // Right Side..
        case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
            Serial.println("FRONT_RIGHT and BACK_RIGHT");
            motor.turnLeft(SPEED);
            delay(1000);
            motor.moveForward(SPEED);
            break;
        // Back Side..
        case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
            motor.moveForward(SPEED);
            Serial.println("BACK_LEFT and BACK_RIGHT");
            break;

        default:
            Serial.println("None");
            return false;
        }

        // switch (state)
        // {
        // /* ------------------------------- One Sensor ------------------------------- */
        // case IRStates::WHITE_FRONT_LEFT:
        //     Serial.println("FRONT_LEFT");
        //     break;
        // case IRStates::WHITE_FRONT_RIGHT:
        //     Serial.println("FRONT_RIGHT");
        //     break;
        // case IRStates::WHITE_BACK_LEFT:
        //     Serial.println("BACK_LEFT");
        //     break;
        // case IRStates::WHITE_BACK_RIGHT:
        //     Serial.println("BACK_RIGHT");
        //     break;

        // /* ------------------------------- Two Sensors ------------------------------ */
        // // Left Side..
        // case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_BACK_LEFT):
        //     Serial.println("FRONT_LEFT and BACK_LEFT");
        //     break;
        // // Front Side..
        // case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
        //     Serial.println("FRONT_LEFT and FRONT_RIGHT");
        //     break;
        // // Right Side..
        // case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
        //     Serial.println("FRONT_RIGHT and BACK_RIGHT");
        //     break;
        // // Back Side..
        // case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
        //     Serial.println("BACK_LEFT and BACK_RIGHT");
        //     break;

        // default:
        //     Serial.println("None");
        //     return false;
        // }

        // switch (state)
        // {
        // /* ------------------------------- One Sensor ------------------------------- */
        // case IRStates::FRONT_LEFT:
        //     controllerMagnets.turnOn();
        //     break;
        // case IRStates::FRONT_RIGHT:
        //     controllerMagnets.turnOn();
        //     break;
        // case IRStates::BACK_LEFT:

        //     break;
        // case IRStates::BACK_RIGHT:
        //     Serial.println("BACK_RIGHT");
        //     break;

        // /* ------------------------------- Two Sensors ------------------------------ */
        // // Left Side..
        // case (IRStates::FRONT_LEFT | IRStates::BACK_LEFT):
        //     Serial.println("FRONT_LEFT and BACK_LEFT");
        //     break;
        // // Front Side..
        // case (IRStates::FRONT_LEFT | IRStates::FRONT_RIGHT):
        //     Serial.println("FRONT_LEFT and FRONT_RIGHT");
        //     break;
        // // Right Side..
        // case (IRStates::FRONT_RIGHT | IRStates::BACK_RIGHT):
        //     Serial.println("FRONT_RIGHT and BACK_RIGHT");
        //     break;
        // // Back Side..
        // case (IRStates::BACK_LEFT | IRStates::BACK_RIGHT):
        //     Serial.println("BACK_LEFT and BACK_RIGHT");
        //     break;

        // default:
        //     Serial.println("None");
        //     return false;
        // }

        return true;
    }
};

/* -------------------------- Controller Instances -------------------------- */
MagnetsController controllerMagnets(MAGNET1_PIN, MAGNET2_PIN, MAGNET3_PIN);
IRController controllerIR(IR_FRONT_LEFT_PIN, IR_FRONT_RIGHT_PIN, IR_BACK_RIGHT_PIN, IR_BACK_LEFT_PIN);

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println();
    char result = controllerIR.checkIRSensors();
    Serial.println((int)result);

    controllerIR.handleIRSensors((IRStates)result);
    delay(250);
}