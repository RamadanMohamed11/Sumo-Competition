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
        if (fabs(g.gyro.x) > 0.02f)
            gyroAngles[Axis::X] += ((g.gyro.x - gyroAverageDrift[Axis::X]) * deltaTime * GYRO_COEFFICIENT) / 1000000.f;
        if (fabs(g.gyro.y) > 0.02f)
            gyroAngles[Axis::Y] += ((g.gyro.y - gyroAverageDrift[Axis::Y]) * deltaTime * GYRO_COEFFICIENT) / 1000000.f;
        if (fabs(g.gyro.z) > 0.02f)
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
    mpu.init();
}

void loop()
{
    mpu.updateReadings();
    // Serial.println(mpu.getGyroAngle(Axis::X));
    delay(50);
}
