#include <Arduino.h>

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

/* ---------------------------- IR Sensor Objects --------------------------- */

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

/* ----------------------- Ultrasonic Sensor Positions ---------------------- */
enum SensorPosition_t
{
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK,
    LEFT,
    RIGHT
};

/* ------------------------ Ultrasonic Sensor Objects ----------------------- */
Ultrasonic frontLeft(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
Ultrasonic frontRight(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
Ultrasonic back(TRIG_BACK, ECHO_BACK);
Ultrasonic left(TRIG_LEFT, ECHO_LEFT);
Ultrasonic right(TRIG_RIGHT, ECHO_RIGHT);

/* -------------------------- Function Declaration -------------------------- */
float measureDistance(SensorPosition_t SensorPosition);
char checkIRSensors();
void handleIRSensors(IRStates state);
void handleUltrasonic();
void turn(int angle);

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    Serial.println();
    char result = controllerIR.checkIRSensors();
    Serial.println((int)result);
    // handleIRSensors((IRStates)result);
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
