#include <Arduino.h>;

/* ----------------------------- Ultrasonic Pins ---------------------------- */
#define US_FR_TRIG 1
#define US_FR_ECHO 1
#define US_FL_TRIG 1
#define US_FL_ECHO 1
#define US_R_TRIG 1
#define US_R_ECHO 1
#define US_L_TRIG 1
#define US_L_ECHO 1
#define US_B_TRIG 1
#define US_B_ECHO 1
/* -------------------------------------------------------------------------- */

/* -------------------------------- Constants ------------------------------- */
// Max distance (in millimeters) an ultrasonic can read. Any distance longer than that is neglected.
#define MAX_DISTANCE 1000

// If the difference in readings between the front right and front left sensors is less than this threshold,
// the robot will stop turning.
// The distance difference is in millimeters.
#define ROTATION_STOP_THRESHOLD 30
/* -------------------------------------------------------------------------- */

/* ---------------------------- Ultrasonic Class ---------------------------- */
class Ultrasonic
{
private:
    int _trigPin;
    int _echoPin;

public:
    // Distance read when there is no object detected in sight.
    // Assigned by ultrasonic controller in the calibration stage.
    double defaultDistance;

    Ultrasonic(int trigPin, int echoPin) { setPins(trigPin, echoPin); }
    Ultrasonic() {}

    /// @brief Assigns the trigger and echo pins and sets up their pin mode.
    void setPins(int trigPin, int echoPin)
    {
        _trigPin = trigPin;
        _echoPin = echoPin;

        pinMode(_trigPin, OUTPUT);
        pinMode(_echoPin, INPUT);
    }

    /// @brief Reads the distance from the ultrasonic sensor.
    /// @return The distance from the sensor in centimeters.
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

enum Index
{
    FR, // Front Right Ultrasonic Sensor Index In The Sensors Array.
    FL, // Front Left Ultrasonic Sensor Index In The Sensors Array.
    B,  // Back Side Ultrasonic Sensor Index In The Sensors Array.
    R,  // Right Side Ultrasonic Sensor Index In The Sensors Array.
    L,  // Left Side Ultrasonic Sensor Index In The Sensors Array.
};

/* -------------------------- Ultrasonic Controller ------------------------- */
class UltrasonicController
{
private:
    Ultrasonic sensors[5];

public:
    UltrasonicController(int FR_TRIG, int FR_ECHO, int FL_TRIG, int FL_ECHO, int R_TRIG, int R_ECHO, int L_TRIG, int L_ECHO, int B_TRIG, int B_ECHO)
    {
        sensors[Index::FR].setPins(FR_TRIG, FR_ECHO);
        sensors[Index::FL].setPins(FL_TRIG, FL_ECHO);
        sensors[Index::R].setPins(R_TRIG, R_ECHO);
        sensors[Index::L].setPins(L_TRIG, L_ECHO);
        sensors[Index::B].setPins(B_TRIG, B_ECHO);
    }

    /// @brief Takes the average of a number of readings done on every sensor.
    /// @param readingCount the number of readings you want to take.
    /// @param delayLength The delay between every reading taken and another.
    void calibrateSensors(int readingCount, int delayLength)
    {

        // Get average readings for all sensors on two stages...
        // The reason we take readings on two different stages is to avoid interference

        // Get average readings for front left, and right side and back sensors.
        for (int i = 0; i < readingCount; i++)
        {
            sensors[Index::FL].defaultDistance += sensors[Index::FL].read();
            sensors[Index::R].defaultDistance += sensors[Index::R].read();
            sensors[Index::B].defaultDistance += sensors[Index::B].read();
            delay(delayLength);
        }

        // Get average readings for front right and left side sensors.
        for (int i = 0; i < readingCount; i++)
        {
            sensors[Index::FR].defaultDistance += sensors[Index::FR].read();
            sensors[Index::L].defaultDistance += sensors[Index::L].read();
            delay(delayLength);
        }

        // Divide sums by the reading count to get the average.
        for (int i = 0; i < 5; i++)
            sensors[i].defaultDistance /= readingCount;
    }

    /// @brief Reads distances from all ultrasonic sensors one by one.
    /// @return returns an array of distances read by the sensors.
    float *getDistances()
    {
        float distances[5] = {};

        // Get a reading from each sensor.
        for (int i = 0; i < 5; i++)
        {
            // Get reading from a sensor.
            float reading = sensors[i].read();

            // If the reading is less than the default distance and the maximum distance, then it's a valid reading.
            if (reading < sensors[i].defaultDistance * 0.9 && reading < MAX_DISTANCE)
                distances[i] = reading;
            // Otherwise the reading is invalid, set the corresponding item in the return array to -1.
            else
                distances[i] = -1;
        }

        return distances;
    }
};

UltrasonicController controller(US_FR_TRIG, US_FR_ECHO, US_FL_TRIG, US_FL_ECHO, US_R_TRIG, US_R_ECHO, US_L_TRIG, US_L_ECHO, US_B_TRIG, US_B_ECHO);

void setup()
{
    Serial.println("SDF");
    controller.calibrateSensors(100, 10);
}

void loop()
{
    float *distances = controller.getDistances();

    // Front Right and Front Left Sensors are reading distances.
    if (distances[Index::FR] != -1 && distances[Index::FL] != -1)
    {
        float distanceDifference = fabs(distances[Index::FR] - distances[Index::FL]);

        if (distanceDifference > ROTATION_STOP_THRESHOLD)
        {
            distanceDifference > 0 ? turnRight() : turnLeft();
        }
        else
        {
            // No need for rotation. Attack!
            moveForward();
        }
    }
    // Front Right Sensor is reading distance.
    // Or
    // Right Side Sensor is reading distance.
    else if (distances[Index::FR] != -1 || distances[Index::R] != -1)
    {
        turnRight();
    }
    // Front Left Sensor is reading distance.
    // Or
    // Left Side Sensor is reading distance.
    else if (distances[Index::FL] != -1 || distances[Index::L] != -1)
    {
        turnLeft();
    }
    // Back Sensor is reading distance.
    else if (distances[Index::B] != -1)
    {
        turnRight();
    }
    // No Sensor is reading a distance.
    else
    {
        // Move forward slowly.
    }
}