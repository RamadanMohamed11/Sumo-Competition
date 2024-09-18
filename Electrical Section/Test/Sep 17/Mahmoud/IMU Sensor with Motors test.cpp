#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* --------------------------- Define Motors Pins --------------------------- */
#define LEFT_MOTOR_P1 15
#define LEFT_MOTOR_P2 2
#define RIGHT_MOTOR_P1 18
#define RIGHT_MOTOR_P2 19

/* ---------------------------- Define Constants ---------------------------- */
#define SPEED 255
#define minDistance 140
#define TURN_SPEED 150
#define numReadings 10

/* ------------------------------- Motor Class ------------------------------ */
class Motor
{
private:
    int _leftMotorPin1;
    int _leftMotorPin2;
    int _rightMotorPin1;
    int _rightMotorPin2;

public:
    /// \brief Constructor for the Motor class.
    /// This constructor sets the pins for the left and right motors and sets them as output pins.
    /// \param leftMotorPin1 The number of the first pin for the left motor.
    /// \param leftMotorPin2 The number of the second pin for the left motor.
    /// \param rightMotorPin1 The number of the first pin for the right motor.
    /// \param rightMotorPin2 The number of the second pin for the right motor.
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

    /// \brief Move the robot forward at the given speed.
    /// This function sets the speed of both the left and right motors to the given value.
    /// \param speed The speed at which to move the robot forward.
    void moveForward(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
    }

    /// \brief Move the robot backward at the given speed.
    /// This function sets the speed of both the left and right motors to the negation of the given value.
    /// \param speed The speed at which to move the robot backward.
    void moveBackward(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
    }

    /// \brief Turn the robot left at the given speed.
    /// This function sets the speed of the left motor to the negation of the given value and the speed of the right motor to the given value.
    /// \param speed The speed at which to turn the robot left.
    void turnLeft(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, -speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, speed);
    }

    /// \brief Turn the robot right at the given speed.
    /// This function sets the speed of the left motor to the given value and the speed of the right motor to the negation of the given value.
    /// \param speed The speed at which to turn the robot right.
    void turnRight(int speed)
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, speed);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, -speed);
    }

    /// \brief Stop the robot from moving.
    /// This function sets the speed of both the left and right motors to 0, which stops the robot from moving.
    void stop()
    {
        setMotorSpeed(_leftMotorPin1, _leftMotorPin2, 0);
        setMotorSpeed(_rightMotorPin1, _rightMotorPin2, 0);
    }

    /// \brief Set the speed of a motor.
    /// This function sets the speed of a motor, given its two pins.
    /// If the speed is positive, the motorPin1 is set to the given value and motorPin2 is set to 0,
    /// if the speed is negative, the motorPin1 is set to 0 and motorPin2 is set to the absolute value of the given speed,
    /// if the speed is 0, both motorPin1 and motorPin2 are set to 0.
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

/* --------------------------------- Motors --------------------------------- */
Motor motors(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

void setup()
{
    Serial.begin(9600);
    mpu.init();
}

void loop()
{
    mpu.updateReadings();

    float previousAngle = mpu.getGyroAngle(Axis::Z);

    float gyroAngle = mpu.getGyroAngle(Axis::Z);

    while (fabs(gyroAngle - previousAngle) < 90)
    {
        mpu.updateReadings();
        gyroAngle = mpu.getGyroAngle(Axis::Z);
        Serial.print("Angle is ");
        Serial.print(gyroAngle);
        Serial.print("  And difference is ");
        Serial.print(fabs(gyroAngle - previousAngle));
        Serial.println();

        motors.turnRight(255);
    }

    motors.stop();
    delay(5000);
}

void turn(int angle)
{
    mpu.updateReadings();
    float current_angle = mpu.getGyroAngle(Axis::Z);
    float target_angle = current_angle + angle;

    while (current_angle != target_angle)
    {
        mpu.updateReadings();
        current_angle = mpu.getGyroAngle(Axis::Z);
        if (angle > 0)
        {
            motors.turnRight(255);
        }
        else
        {
            motors.turnLeft(255);
        }
    }

    motors.stop();
}