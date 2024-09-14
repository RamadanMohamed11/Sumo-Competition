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

    /* ############################# State Variables ############################ */
    long long lastTime = 0;
    long long deltaT = 0;

    /* ################################# Angles ################################# */
    float x_angle = 0;
    float y_angle = 0;
    float z_angle = 0;

    float gyro_x_angle = 0;
    float gyro_y_angle = 0;
    float gyro_z_angle = 0;

    float uncertainty = 0;

    // When you multiply this coefficient by the angular velocity reading,
    // you get angular velocity in degrees per second.
    const double ANGULAR_VELOCITY_COEFFICIENT = 57.2956455309;

public:
    MPU_Sensor()
        : mpu()
    {
    }

    void init()
    {
        // Try to initialize!
        if (!mpu.begin())
        {
            Serial.println("Failed to find MPU6050 chip");
            while (1)
            {
                Serial.println("Error");
                delay(10);
            }
        }

        // The maximum acceleration will never exceed 2G.
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

        // The rotation is in the range of 500 degrees.
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);

        // No idea ⚆_⚆
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // Updates readings and changes the values of the private members.
    void updateReadingsKalman()
    {
        // Get new sensor readings.
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Accelerometer readings...
        float theta = atan2(a.acceleration.x / 9.8f, a.acceleration.z / 9.8f) / 2 / 3.141592654;
        float phi = atan2(a.acceleration.y / 9.8f, a.acceleration.z / 9.8f) / 2 / 3.141592654;

        // Logging...
        Serial.print(theta);
        Serial.print(",");
        Serial.print(phi);
        Serial.println();

        // Update time...
        deltaT = (micros() - this->lastTime);
        lastTime = micros();

        // Gyroscope readings...
        // Get angles by multiplying angular velocity by deltaT.
        gyro_x_angle += (g.gyro.x * deltaT * ANGULAR_VELOCITY_COEFFICIENT) / 1000000.f;
        gyro_y_angle += (g.gyro.y * deltaT * ANGULAR_VELOCITY_COEFFICIENT) / 1000000.f;
        gyro_z_angle += (g.gyro.z * deltaT * ANGULAR_VELOCITY_COEFFICIENT) / 1000000.f;

        /* ########################## Complementary Filter ########################## */
        // Final Angle..
        x_angle = 0.9 * theta + 0.1 * gyro_x_angle;
        y_angle = 0.9 * phi + 0.1 * gyro_y_angle;

        // // Cap angles at 360 degrees...
        // x_angle > 360 ? x_angle -= 360 : (x_angle < -360 ? x_angle += 360 : x_angle);
        // y_angle > 360 ? y_angle -= 360 : (y_angle < -360 ? y_angle += 360 : y_angle);
        // z_angle > 360 ? z_angle -= 360 : (z_angle < -360 ? z_angle += 360 : z_angle);
    }

    // Updates readings and changes the values of the private members.
    void updateReadings()
    {
        // Get new sensor readings.
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Accelerometer readings...
        float theta = atan2(a.acceleration.x / 9.8f, a.acceleration.z / 9.8f) / 2 / 3.141592654;
        float phi = atan2(a.acceleration.y / 9.8f, a.acceleration.z / 9.8f) / 2 / 3.141592654;

        // Logging...
        // Serial.print(theta);
        // Serial.print(",");
        // Serial.print(phi);
        // Serial.println();

        // Update time...
        deltaT = (micros() - this->lastTime);
        lastTime = micros();

        // Gyroscope readings...
        // Get angles by multiplying angular velocity by deltaT.
        gyro_x_angle += (g.gyro.x * deltaT * ANGULAR_VELOCITY_COEFFICIENT) / 1000000.f;
        gyro_y_angle += (g.gyro.y * deltaT * ANGULAR_VELOCITY_COEFFICIENT) / 1000000.f;
        float delta = (g.gyro.z * deltaT * ANGULAR_VELOCITY_COEFFICIENT) / 1000000.f;
        if (fabs(delta) > 0.5f)
            gyro_z_angle += delta;

        /* ########################## Complementary Filter ########################## */
        // Final Angle..
        x_angle = 0.9 * theta + 0.1 * gyro_x_angle;
        y_angle = 0.9 * phi + 0.1 * gyro_y_angle;

        Serial.print(gyro_x_angle);
        Serial.print(",");
        Serial.print(gyro_y_angle);
        Serial.print(",");
        Serial.print(gyro_z_angle);
        Serial.println();

        // // Cap angles at 360 degrees...
        // x_angle > 360 ? x_angle -= 360 : (x_angle < -360 ? x_angle += 360 : x_angle);
        // y_angle > 360 ? y_angle -= 360 : (y_angle < -360 ? y_angle += 360 : y_angle);
        // z_angle > 360 ? z_angle -= 360 : (z_angle < -360 ? z_angle += 360 : z_angle);
    }

    // Gets the angle of rotation on a certain axis.
    float getAngle(Axis axis)
    {
        switch (axis)
        {
        case Axis::X:
            return this->x_angle;
        case Axis::Y:
            return this->y_angle;
        case Axis::Z:
            return this->z_angle;
        }
    }
};

/* ################################### Mpu ################################## */
MPU_Sensor mpu;

void setup()
{
    Serial.begin(9600);
    mpu.init();
}

void loop()
{
    mpu.updateReadings();
    Serial.println(mpu.getAngle(Axis::X));
    delay(50);
}
