#include <Arduino.h>
#include <BluetoothSerial.h>

/* ------------------------------- Motors Pins ------------------------------ */
#pragma region Motors Pins

#define LEFT_MOTOR_P1 32
#define LEFT_MOTOR_P2 33
#define RIGHT_MOTOR_P1 18
#define RIGHT_MOTOR_P2 19

#pragma endregion

/* -------------------------------- Constants ------------------------------- */
#pragma region Constants

#define SPEED 255
#define MAX_SPEED 8191
#define minDistance 140
#define TURN_SPEED 150

#pragma endregion

/* ----------------------------- IR Sensor Pins ----------------------------- */
#pragma region IR Sensor Pins

#define IR_FRONT_LEFT_PIN 4
#define IR_FRONT_RIGHT_PIN 5
#define IR_BACK_LEFT_PIN 19
#define IR_BACK_RIGHT_PIN 21

#pragma endregion

/* ---------------------------- Bluetooth Serial ---------------------------- */
BluetoothSerial SerialBT;

/* ------------------------------ Movement Type ----------------------------- */
enum Movement_Type
{
    TURNING_LEFT,
    MOVING_FORWARD,
    TURNING_RIGHT,
    MOVING_BACKWARD,
};

// Constants.
#define REACHED_THRESHOLD 1

/* ------------------------------- Motor Class ------------------------------ */
class Motor
{
private:
    // Pins.
    int _leftMotorPin1;
    int _leftMotorPin2;
    int _rightMotorPin1;
    int _rightMotorPin2;

    // PID parameters.
    float kp;
    float ki;
    float kd;

    // PID calculation variables.
    long long int previousTime;
    long long int previousError;
    float prevIntegral;

    // State variables.
    bool targetReached;
    Movement_Type movementType;

public:
    float distanceFactor;

    // State variables.
    double current;
    double target;

    Motor(int leftMotorPin1, int leftMotorPin2, int rightMotorPin1, int rightMotorPin2)
    {
        // Assign pins.
        _leftMotorPin1 = leftMotorPin1;
        _leftMotorPin2 = leftMotorPin2;
        _rightMotorPin1 = rightMotorPin1;
        _rightMotorPin2 = rightMotorPin2;

        // Initialize pins.
        pinMode(_leftMotorPin1, OUTPUT);
        pinMode(_leftMotorPin2, OUTPUT);
        pinMode(_rightMotorPin1, OUTPUT);
        pinMode(_rightMotorPin2, OUTPUT);

        // Assign PID parameters.
        this->kp = 1;
        this->ki = 0;
        this->kd = 0.025;

        // Initialize PID calculation variables.
        this->previousTime = micros();
        this->previousError = 0;
        this->prevIntegral = 0;

        // Initialize state variables.
        this->current = 0;
        this->target = 0;
        this->targetReached = false;
        this->movementType = Movement_Type::MOVING_FORWARD;
    }

#pragma region PID Functions

    /// @brief Changes the PID parameters.
    /// @param kp Proportional constant.
    /// @param ki Integral constant.
    /// @param kd Differential constant.
    void setupPID(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    /// @brief Changes the speed on the motors based on the progress it achieved.
    /// MUST BE CALLED IN LOOP TO ACHIEVE ACCURATE MOVEMENT.
    bool updateSignal(double newPos = 0)
    {
        if (targetReached)
            return true;
#pragma region PID Calculations

        // Get new position.
        current = newPos ? newPos : current;

        // Calculate delta T and update the previous time term (previousTime).
        long long int currentTime = micros();
        double deltaT = (((double)(currentTime - previousTime)) / 1000000);
        previousTime = currentTime;

        // Calculate the error.
        double error = current - target;

        // Calculate the derivative term and update the previous error term (previousError).
        float derivative = (error - previousError) / deltaT;
        previousError = error;

        // Calculate the integral term.
        prevIntegral += error * deltaT;

        // Calculate the PID signal.
        float signal = kp * error + kd * derivative + ki * prevIntegral;

        // Calculate speed.
        float speed = fabs(signal);
        if (speed > MAX_SPEED)
        {
            speed = MAX_SPEED;
        }
#pragma endregion

        // Apply signal...
        switch (movementType)
        {
        case Movement_Type::TURNING_LEFT:
            setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
            setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
            break;
        case Movement_Type::MOVING_FORWARD:
            setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
            setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
            break;
        case Movement_Type::TURNING_RIGHT:
            setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
            setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
            break;
        case Movement_Type::MOVING_BACKWARD:
            setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
            setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
            break;
        }

        // Check if the motor has reached its target.
        targetReached = previousError < REACHED_THRESHOLD && previousError > -REACHED_THRESHOLD;

        // If it reached the target, stop the motors.
        if (targetReached)
            stop();

        // LOGGING...
        SerialBT.print("Current= ");
        SerialBT.print(current);
        SerialBT.print(", Target= ");
        SerialBT.print(target);
        SerialBT.println();

        return false;
    }

    //! Under Work...
    /// @brief
    /// @param target
    void moveTo(float target)
    {
        this->target = target;
    }

    /// @brief Turns the robot with PID control.
    /// @param currentAngle The current heading.
    /// @param angle The angle difference (90 to turn right 90 degrees and -90 to turn left).
    void turnPID(float currentAngle, float angle)
    {
        current = currentAngle;
        target = currentAngle + angle;
        // kp = fabs((float)MAX_SPEED / angle) * 0.75;
        kp = 3;
        movementType = angle > 0 ? Movement_Type::TURNING_RIGHT : Movement_Type::TURNING_LEFT;
        targetReached = false;
    }

    // Sets current to 0.
    void reset()
    {
        current = 0;
        targetReached = false;
    }

#pragma endregion

#pragma region normal functions

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

#pragma endregion
};

/* --------------------------------- Motors --------------------------------- */
Motor motors(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

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
            SerialBT.println("FRONT_LEFT");
            break;
        case IRStates::WHITE_FRONT_RIGHT:
            SerialBT.println("FRONT_RIGHT");
            break;
        case IRStates::WHITE_BACK_LEFT:
            SerialBT.println("BACK_LEFT");
            break;
        case IRStates::WHITE_BACK_RIGHT:
            SerialBT.println("BACK_RIGHT");
            break;

        /* ------------------------------- Two Sensors ------------------------------ */
        // Left Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_BACK_LEFT):
            SerialBT.println("FRONT_LEFT and BACK_LEFT");
            break;
        // Front Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
            SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
            break;
        // Right Side..
        case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
            SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
            break;
        // Back Side..
        case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
            SerialBT.println("BACK_LEFT and BACK_RIGHT");
            break;

        // None are detecting white..
        case IRStates::NO_WHITE:
            SerialBT.println("None");
            return false;
            break;

            // Impossible case.
            // default:
            SerialBT.println("None");
        }

        return true;
    }

    bool handleIRSensors(IRStates state)
    {

        switch (state)
        {
        /* ------------------------------- One Sensor ------------------------------- */
        case IRStates::WHITE_FRONT_LEFT:
            SerialBT.println("FRONT_LEFT");
            motors.moveBackward(SPEED);
            delay(500);
            motors.turnRight(SPEED);
            delay(500);
            motors.moveForward(SPEED);
            delay(500);
            break;
        case IRStates::WHITE_FRONT_RIGHT:
            SerialBT.println("FRONT_RIGHT");
            motors.moveBackward(SPEED);
            delay(500);
            motors.turnLeft(SPEED);
            delay(500);
            motors.moveForward(SPEED);
            delay(500);
            break;
        case IRStates::WHITE_BACK_LEFT:
            SerialBT.println("BACK_LEFT");
            motors.moveForward(SPEED);
            delay(500);
            motors.turnRight(SPEED);
            delay(500);
            motors.moveBackward(SPEED);
            delay(500);
            break;
        case IRStates::WHITE_BACK_RIGHT:
            SerialBT.println("BACK_RIGHT");
            motors.moveForward(SPEED);
            delay(500);
            motors.turnLeft(SPEED);
            delay(500);
            motors.moveBackward(SPEED);
            delay(500);
            break;

        /* ------------------------------- Two Sensors ------------------------------ */
        // Left Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_BACK_LEFT):
            SerialBT.println("FRONT_LEFT and BACK_LEFT");
            motors.turnRight(SPEED);
            delay(500);
            motors.moveForward(SPEED);
            delay(500);
            break;
        // Front Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
            SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
            motors.moveBackward(SPEED);
            delay(500);
            break;
        // Right Side..
        case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
            SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
            motors.turnLeft(SPEED);
            delay(500);
            motors.moveForward(SPEED);
            delay(500);
            break;
        // Back Side..
        case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
            SerialBT.println("BACK_LEFT and BACK_RIGHT");
            motors.moveForward(SPEED);
            delay(500);
            break;

        default:
            SerialBT.println("None");
            return false;
        }

        motors.stop();
        return true;
    }
};

/* -------------------------- Controller Instances -------------------------- */
// MagnetsController controllerMagnets(MAGNET1_PIN, MAGNET2_PIN, MAGNET3_PIN);
IRController controllerIR(IR_FRONT_LEFT_PIN, IR_FRONT_RIGHT_PIN, IR_BACK_RIGHT_PIN, IR_BACK_LEFT_PIN);

void setup()
{
    SerialBT.begin("ESP32_Robot");
}

void loop()
{
#pragma region motors test

    // motor.moveForward(SPEED);
    // SerialBT.println("Moving Forward");
    // delay(2000);
    // motor.moveBackward(SPEED);
    // SerialBT.println("Moving Backward");
    // delay(2000);
    // motor.turnRight(SPEED);
    // SerialBT.println("Turning Right");
    // delay(2000);
    // motor.turnLeft(SPEED);
    // SerialBT.println("Turning Left");
    // delay(2000);
    // // char result = controllerIR.checkIRSensors();
    // SerialBT.println((int)result);

#pragma endregion

    char result = controllerIR.checkIRSensors();
    controllerIR.handleIRSensors((IRStates)result);
}
