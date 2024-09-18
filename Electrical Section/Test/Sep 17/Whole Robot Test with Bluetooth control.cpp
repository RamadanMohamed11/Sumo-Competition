#include <Arduino.h>
#include <BluetoothSerial.h>

/* ------------------------- Define Ultrasonics Pins ------------------------ */
#pragma region Define Ultrasonics Pins

// Front Left Sensor.
#define TRIG_FRONT_LEFT 26
#define ECHO_FRONT_LEFT 25

// Front Right Sensor.
#define TRIG_FRONT_RIGHT 7
#define ECHO_FRONT_RIGHT 8

// Right Sensor.
#define TRIG_RIGHT 23
#define ECHO_RIGHT 24

// Left Sensor.
#define TRIG_LEFT 27
#define ECHO_LEFT 28

// Back Sensor.
#define TRIG_BACK 6
#define ECHO_BACK 5

#pragma endregion

/* ---------------------------- Define Motors Pis ---------------------------- */
#pragma region Define Motors Pins

#define LEFT_MOTOR_P1 32
#define LEFT_MOTOR_P2 33
#define RIGHT_MOTOR_P1 18
#define RIGHT_MOTOR_P2 19

#pragma endregion

/* ---------------------------- Define Constants ---------------------------- */
#pragma region Define Constants

#define SPEED 255
#define minDistance 140
#define TURN_SPEED 150

#pragma endregion

/* ------------------------------- Magnets Pins ------------------------------ */
#pragma region Magnets Pins

#define MAGNET1_PIN 13
#define MAGNET2_PIN 2
#define MAGNET3_PIN 3
#define MAGNET_COUNT 3

#pragma endregion

/* -------------------------- Define IR Sensor Pins ------------------------- */
#pragma region Define IR Sensor Pins

#define IR_FRONT_LEFT_PIN 4
#define IR_FRONT_RIGHT_PIN 5
#define IR_BACK_LEFT_PIN 19
#define IR_BACK_RIGHT_PIN 21

#pragma endregion

/* ---------------------------- Bluetooth Serial ---------------------------- */
BluetoothSerial SerialBT;
volatile char command = '0';

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
        stop();
        SerialBT.print("MOVE FORWARD, SPEED is ");
        SerialBT.println(speed);
        delay(50);
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
        delay(15);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
    }

    void moveBackward(int speed)
    {
        stop();
        delay(50);
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
        delay(15);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
    }

    void turnLeft(int speed)
    {
        stop();
        delay(50);
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
        delay(15);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
    }

    void turnRight(int speed)
    {
        stop();
        delay(50);
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
        delay(15);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
    }

    void stop()
    {
        SerialBT.println("STOPPING...");
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, 0);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, 0);
    }

    void setMotorSpeed(int motorPin1, int motorPin2, int speed)
    {
        if (motorPin1 == LEFT_MOTOR_P1)
            SerialBT.print("LEFT Motor, ");
        else
            SerialBT.print("RIGHT Motor, ");

        if (speed > 0)
        {
            analogWrite(motorPin1, speed);
            analogWrite(motorPin2, 0);

            SerialBT.print("  Positive speed, ");
            SerialBT.print(speed);
            SerialBT.println();
        }
        else if (speed < 0)
        {
            analogWrite(motorPin1, 0);
            analogWrite(motorPin2, -speed);

            SerialBT.print("  Negative speed, ");
            SerialBT.print(speed);
            SerialBT.println();
        }
        else
        {
            SerialBT.print("  NO SPEED.");
            SerialBT.print(speed);
            SerialBT.println();
            analogWrite(motorPin1, 0);
            analogWrite(motorPin2, 0);
        }

        SerialBT.println();
    }
};

/* ------------------------------ Motor Object ------------------------------ */
Motor motors(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

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
    NO_WHITE = 0b0000,
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

    bool handleIRSensors_DEBUG(IRStates state)
    {
        switch (state)
        {
        /* ------------------------------- One Sensor ------------------------------- */
        case IRStates::WHITE_FRONT_LEFT:
            // SerialBT.println("FRONT_LEFT");
            break;
        case IRStates::WHITE_FRONT_RIGHT:
            // SerialBT.println("FRONT_RIGHT");
            break;
        case IRStates::WHITE_BACK_LEFT:
            // SerialBT.println("BACK_LEFT");
            break;
        case IRStates::WHITE_BACK_RIGHT:
            // SerialBT.println("BACK_RIGHT");
            break;

        /* ------------------------------- Two Sensors ------------------------------ */
        // Left Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_BACK_LEFT):
            // SerialBT.println("FRONT_LEFT and BACK_LEFT");
            break;
        // Front Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
            // SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
            break;
        // Right Side..
        case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
            // SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
            break;
        // Back Side..
        case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
            // SerialBT.println("BACK_LEFT and BACK_RIGHT");
            break;

        // None are detecting white..
        case IRStates::NO_WHITE:
            // SerialBT.println("None");
            return false;
            break;

            // Impossible case.
            // default:
            // SerialBT.println("None");
        }

        return true;
    }

    bool handleIRSensors(IRStates state)
    {

        switch (state)
        {
        /* ------------------------------- One Sensor ------------------------------- */
        case IRStates::WHITE_FRONT_LEFT:
            // SerialBT.println("FRONT_LEFT");
            break;
        case IRStates::WHITE_FRONT_RIGHT:
            // SerialBT.println("FRONT_RIGHT");
            break;
        case IRStates::WHITE_BACK_LEFT:
            // SerialBT.println("BACK_LEFT");
            break;
        case IRStates::WHITE_BACK_RIGHT:
            // SerialBT.println("BACK_RIGHT");
            break;

        /* ------------------------------- Two Sensors ------------------------------ */
        // Left Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_BACK_LEFT):
            // SerialBT.println("FRONT_LEFT and BACK_LEFT");
            motors.turnRight(SPEED);
            delay(1000);
            motors.moveForward(SPEED);
            break;
        // Front Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
            // SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
            motors.moveBackward(SPEED);
            break;
        // Right Side..
        case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
            // SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
            motors.turnLeft(SPEED);
            delay(1000);
            motors.moveForward(SPEED);
            break;
        // Back Side..
        case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
            motors.moveForward(SPEED);
            // SerialBT.println("BACK_LEFT and BACK_RIGHT");
            break;

        default:
            // SerialBT.println("None");
            return false;
        }

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
        //     SerialBT.println("BACK_RIGHT");
        //     break;

        // /* ------------------------------- Two Sensors ------------------------------ */
        // // Left Side..
        // case (IRStates::FRONT_LEFT | IRStates::BACK_LEFT):
        //     SerialBT.println("FRONT_LEFT and BACK_LEFT");
        //     break;
        // // Front Side..
        // case (IRStates::FRONT_LEFT | IRStates::FRONT_RIGHT):
        //     SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
        //     break;
        // // Right Side..
        // case (IRStates::FRONT_RIGHT | IRStates::BACK_RIGHT):
        //     SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
        //     break;
        // // Back Side..
        // case (IRStates::BACK_LEFT | IRStates::BACK_RIGHT):
        //     SerialBT.println("BACK_LEFT and BACK_RIGHT");
        //     break;

        // default:
        //     SerialBT.println("None");
        //     return false;
        // }

        return true;
    }
};

/* -------------------------- Controller Instances -------------------------- */
// MagnetsController controllerMagnets(MAGNET1_PIN, MAGNET2_PIN, MAGNET3_PIN);
// IRController controllerIR(IR_FRONT_LEFT_PIN, IR_FRONT_RIGHT_PIN, IR_BACK_RIGHT_PIN, IR_BACK_LEFT_PIN);

void setup()
{
    SerialBT.begin("ESP32_Robot");
}

#define MAX_SPEED 8191
void loop()
{
    if (SerialBT.available())
    {
        command = SerialBT.read();
        switch (command)
        {
        case 'f':

            motors.moveForward(MAX_SPEED);
            break;
        case 'b':

            motors.moveBackward(MAX_SPEED);
            break;
        case 'r':

            motors.turnRight(MAX_SPEED);
            break;
        case 'l':

            motors.turnLeft(MAX_SPEED);
            break;
        case 's':

            motors.stop();
            break;
        }

        command = '\0';
    }
}
