#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* ---------------------------- Ultrasonics Pins ---------------------------- */
#pragma region Ultrasonics Pins

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

/* ------------------------------ Magnets Pins ------------------------------ */
#pragma region Magnets Pins

#define MAGNET1_PIN 13
#define MAGNET2_PIN 2
#define MAGNET3_PIN 3
#define MAGNET_COUNT 3

#pragma endregion

/* ----------------------------- IR Sensor Pins ----------------------------- */
#pragma region IR Sensor Pins

#define IR_FRONT_LEFT_PIN 4
#define IR_FRONT_RIGHT_PIN 5
#define IR_BACK_LEFT_PIN 19
#define IR_BACK_RIGHT_PIN 21

#pragma endregion

/* -------------------------------- Axis Enum ------------------------------- */
enum Axis
{
    X,
    Y,
    Z,
};

/* ------------------------------ MPU6050 CLASS ----------------------------- */
class MPU_Sensor
{
private:
    Adafruit_MPU6050 mpu;

    /* ----------------------------- State Variables ---------------------------- */
    long long lastTime = 0;

    /* ------------------------------ Accelerometer ----------------------------- */
    double accelerometerAngles[3];
    double accelerometerDrift[3];

    /* ---------------------------------- Gyro ---------------------------------- */
    double gyroAverageDrift[3];
    double gyroDriftMagnitude[3];
    double gyroAngles[3];

    // When you multiply this coefficient by the angular velocity reading,
    // you get angular velocity in degrees per second.
    const double GYRO_COEFFICIENT = 57.2956455309;

public:
    MPU_Sensor()
        : mpu()
    {
    }

    void init()
    {
        // Try to initialize!
        while (!mpu.begin())
        {
            Serial.println("Failed to find MPU6050 chip");
            delay(500);
        }

        // The maximum acceleration will never exceed 2G.
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

        // The rotation is in the range of 500 degrees.
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);

        // No idea ⚆_⚆
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

        // Logging.
        Serial.println("MPU Initialization finished...");
        Serial.println("Please wait for calibration process to finish..");
        Serial.println("Do NOT move the sensor.");

        // Calibrate.
        getAverageDrift(300, 10);

        // Log.
        Serial.println("Calibration process finished successfully.");
    }

    void getAverageDrift(int testCount, int delayLength)
    {
        for (int i = 0; i < testCount; i++)
        {
            // Get new sensor readings.
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            // Measure average gyroscope drifts..
            gyroAverageDrift[Axis::X] += g.gyro.x;
            gyroAverageDrift[Axis::Y] += g.gyro.y;
            gyroAverageDrift[Axis::Z] += g.gyro.z;

            // Measure average magnitude of gyroscope drifts.
            gyroDriftMagnitude[Axis::X] += fabs(g.gyro.x);
            gyroDriftMagnitude[Axis::Y] += fabs(g.gyro.y);
            gyroDriftMagnitude[Axis::Z] += fabs(g.gyro.z);

            delay(delayLength);
        }

        // Divide summation by test count to get average.
        for (int i = 0; i < 3; i++)
        {
            gyroAverageDrift[i] /= testCount;
            gyroDriftMagnitude[i] /= testCount;
        }
    }

    void getAccelerometerAngles(sensors_event_t a, sensors_event_t g, long long deltaTime)
    {
        // Accelerometer readings...
        float x_acc = a.acceleration.x - accelerometerDrift[Axis::X];
        float y_acc = a.acceleration.y - accelerometerDrift[Axis::Y];
        float z_acc = a.acceleration.z - accelerometerDrift[Axis::Z];

        // Angle calculation with trigonometry.
        accelerometerAngles[Axis::X] = atan2(x_acc, z_acc) * 180 / 3.141592654;
        accelerometerAngles[Axis::Y] = atan2(y_acc, z_acc) * 180 / 3.141592654;

        // Logging...
        Serial.print("Accelerometer X angle = ");
        Serial.print(accelerometerAngles[Axis::X]);
        Serial.print(", ");

        Serial.print("Accelerometer Y angle = ");
        Serial.print(accelerometerAngles[Axis::Y]);
        Serial.print(", ");
    }

    void getGyroscopeAngles(sensors_event_t a, sensors_event_t g, long long deltaTime, bool ignoreSmallValues)
    {
        // Gyroscope readings...
        // First subtract the average drift angle from the angular velocity (g.gyro.x - gyroAverageDrift[Axis::X])
        // Then multiply by deltaTime in milliseconds to convert angular velocity to angle change.
        // Multiply by GYRO_COEFFICIENT to get get reading in degrees.
        // Finally add that angle change to the previous angle reading.
        if (fabs(g.gyro.x) > gyroDriftMagnitude[Axis::X] && ignoreSmallValues)
            gyroAngles[Axis::X] += ((g.gyro.x - gyroAverageDrift[Axis::X]) * deltaTime * GYRO_COEFFICIENT) / 1000000.f;
        if (fabs(g.gyro.y) > gyroDriftMagnitude[Axis::Y] && ignoreSmallValues)
            gyroAngles[Axis::Y] += ((g.gyro.y - gyroAverageDrift[Axis::Y]) * deltaTime * GYRO_COEFFICIENT) / 1000000.f;
        if (fabs(g.gyro.z) > gyroDriftMagnitude[Axis::Z] && ignoreSmallValues)
            gyroAngles[Axis::Z] += ((g.gyro.z - gyroAverageDrift[Axis::Z]) * deltaTime * GYRO_COEFFICIENT) / 1000000.f;

        // Logging...
        Serial.print("Gyro X = ");
        Serial.print(gyroAngles[Axis::X]);
        Serial.print(", ");

        Serial.print("Gyro Y = ");
        Serial.print(gyroAngles[Axis::Y]);
        Serial.print(", ");

        Serial.print("Gyro Z = ");
        Serial.print(gyroAngles[Axis::Z]);
        Serial.print(", ");
        Serial.println();
    }

    void updateReadings()
    {
        // Get new sensor readings.
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Update time...
        long long deltaTime = (micros() - this->lastTime);
        lastTime = micros();

        // Get readings...
        getGyroscopeAngles(a, g, deltaTime, true);
        // getAccelerometerAngles(a, g, deltaTime);
    }

    // Gets the angle of rotation on a certain axis.
    double getGyroAngle(Axis axis)
    {
        return gyroAngles[axis];
    }
};

/* ----------------------------------- Mpu ---------------------------------- */
MPU_Sensor mpu;

// void loop()
// {
//     mpu.updateReadings();

//     float previousAngle = mpu.getGyroAngle(Axis::Z);

//     float gyroAngle = mpu.getGyroAngle(Axis::Z);

//     while (fabs(gyroAngle - previousAngle) < 90)
//     {
//         mpu.updateReadings();
//         gyroAngle = mpu.getGyroAngle(Axis::Z);
//         Serial.print("Angle is ");
//         Serial.print(gyroAngle);
//         Serial.print("  And difference is ");
//         Serial.print(fabs(gyroAngle - previousAngle));
//         Serial.println();

//         motors.turnRight(255);
//     }

//     motors.stop();
//     delay(5000);
// }

// void turn(int angle)
// {
//     mpu.updateReadings();
//     float current_angle = mpu.getGyroAngle(Axis::Z);
//     float target_angle = current_angle + angle;

//     while (current_angle != target_angle)
//     {
//         mpu.updateReadings();
//         current_angle = mpu.getGyroAngle(Axis::Z);
//         if (angle > 0)
//         {
//             motors.turnRight(255);
//         }
//         else
//         {
//             motors.turnLeft(255);
//         }
//     }

//     motors.stop();
// }

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
            motors.turnRight(SPEED);
            delay(1000);
            motors.moveForward(SPEED);
            break;
        // Front Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
            SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
            motors.moveBackward(SPEED);
            break;
        // Right Side..
        case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
            SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
            motors.turnLeft(SPEED);
            delay(1000);
            motors.moveForward(SPEED);
            break;
        // Back Side..
        case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
            motors.moveForward(SPEED);
            SerialBT.println("BACK_LEFT and BACK_RIGHT");
            break;

        default:
            SerialBT.println("None");
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
        SerialBT.println("BACK_RIGHT");
        //     break;

        // /* ------------------------------- Two Sensors ------------------------------ */
        // // Left Side..
        // case (IRStates::FRONT_LEFT | IRStates::BACK_LEFT):
        SerialBT.println("FRONT_LEFT and BACK_LEFT");
        //     break;
        // // Front Side..
        // case (IRStates::FRONT_LEFT | IRStates::FRONT_RIGHT):
        SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
        //     break;
        // // Right Side..
        // case (IRStates::FRONT_RIGHT | IRStates::BACK_RIGHT):
        SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
        //     break;
        // // Back Side..
        // case (IRStates::BACK_LEFT | IRStates::BACK_RIGHT):
        SerialBT.println("BACK_LEFT and BACK_RIGHT");
        //     break;

        // default:
        SerialBT.println("None");
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

    mpu.init();
    mpu.updateReadings();

    float angle = mpu.getGyroAngle(Axis::Z);
    motors.turnPID(angle, 90);
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

    // // controllerIR.handleIRSensors((IRStates)result);

    mpu.updateReadings();
    float angle = mpu.getGyroAngle(Axis::Z);

    if (motors.updateSignal(angle))
    {
        motors.stop();
        delay(5000);
        angle = mpu.getGyroAngle(Axis::Z);
        motors.turnPID(angle, 90);
    }
}
