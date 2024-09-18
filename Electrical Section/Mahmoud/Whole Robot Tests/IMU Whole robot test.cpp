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
        default:
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
            motor.turnRight(SPEED);
            delay(1000);
            motor.moveForward(SPEED);
            break;
        // Front Side..
        case (IRStates::WHITE_FRONT_LEFT | IRStates::WHITE_FRONT_RIGHT):
            SerialBT.println("FRONT_LEFT and FRONT_RIGHT");
            motor.moveBackward(SPEED);
            break;
        // Right Side..
        case (IRStates::WHITE_FRONT_RIGHT | IRStates::WHITE_BACK_RIGHT):
            SerialBT.println("FRONT_RIGHT and BACK_RIGHT");
            motor.turnLeft(SPEED);
            delay(1000);
            motor.moveForward(SPEED);
            break;
        // Back Side..
        case (IRStates::WHITE_BACK_LEFT | IRStates::WHITE_BACK_RIGHT):
            motor.moveForward(SPEED);
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
MagnetsController controllerMagnets(MAGNET1_PIN, MAGNET2_PIN, MAGNET3_PIN);
IRController controllerIR(IR_FRONT_LEFT_PIN, IR_FRONT_RIGHT_PIN, IR_BACK_RIGHT_PIN, IR_BACK_LEFT_PIN);

void setup()
{
    SerialBT.begin("ESP32_Robot");
    mpu.init();
}

void loop()
{

#pragma region Motors Test With Bluetooth Serial Messages

    motor.moveForward(SPEED);
    SerialBT.println("Moving Forward");
    delay(2000);
    motor.moveBackward(SPEED);
    SerialBT.println("Moving Backward");
    delay(2000);
    motor.turnRight(SPEED);
    SerialBT.println("Turning Right");
    delay(2000);
    motor.turnLeft(SPEED);
    SerialBT.println("Turning Left");
    delay(2000);
    char result = controllerIR.checkIRSensors();
    SerialBT.println((int)result);

#pragma endregion

    mpu.updateReadings();
    // Serial.println(mpu.getGyroAngle(Axis::X));
    delay(50);

    controllerIR.handleIRSensors((IRStates)result);
    delay(250);
}

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

enum Axis
{
    X,
    Y,
    Z,
};

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

void setup()
{
    Serial.begin(9600);
}

void loop()
{
}
