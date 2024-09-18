#include <Arduino.h>
// GO TO LINE 213 TO TEST THE CODE..

// Forward declaration of the Motor class
class Motor;

// Define the readEncoder function after the Motor class is fully defined
void readEncoder(Motor *m);

class Motor
{
private:
    // Pins.
    byte enablePin;
    byte input1;
    byte input2;

    // PID parameters.
    float kp;
    float ki;
    float kd;

    // PID calculation variables.
    long long int previousTime;
    long long int previousError;
    float prevIntegral;

    // State variables.
    bool reverseDirection;
    bool targetReached;

public:
    // Pins.
    byte encoderA;
    byte encoderB;

    float distanceFactor;

    // State variables.
    long long int position;
    long long int target;

    Motor(byte enablePin, byte input1, byte input2, byte encoderA, byte encoderB, float distanceFactor)
    {
        // Initialize pins.
        this->enablePin = enablePin;
        this->input1 = input1;
        this->input2 = input2;
        this->encoderA = encoderA;
        this->encoderB = encoderB;

        // Distance calculation factor.
        // Whatever distance you ask the motor to move to will be multiplied by this factor.
        // For calibration purposes...
        this->distanceFactor = 1;

        // Initialize PID parameters.
        this->kp = 1;
        this->ki = 0;
        this->kd = 0.025;

        // Initialize PID calculation variables.
        this->previousTime = micros();
        this->previousError = 0;
        this->prevIntegral = 0;

        // Initialize state variables.
        this->position = 0;
        this->target = 0;
        this->targetReached = false;
        this->reverseDirection = false;
        setDirection(false);

        // Set the pin modes.
        pinMode(this->enablePin, OUTPUT);
        pinMode(this->input1, OUTPUT);
        pinMode(this->input2, OUTPUT);
        pinMode(this->encoderA, INPUT_PULLUP);
        pinMode(this->encoderB, INPUT_PULLUP);
    }

    // Changes the PID parameters.
    void setupPID(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    // Sets the speed of the motor with a value ranging from 0 to 255.
    void setSpeed(byte speed)
    {
        analogWrite(this->enablePin, speed);
    }

    // Sets the speed of the motor with a value ranging from 0 to 255. Also sets the direction.
    void setSpeed(byte speed, bool reverse)
    {
        setDirection(reverse);
        setSpeed(speed);
    }

    // Sets the rotational direction.
    void setDirection(bool reverse)
    {
        this->reverseDirection = reverse;
        digitalWrite(input1, reverse ? LOW : HIGH);
        digitalWrite(input2, reverse ? HIGH : LOW);
    }

    // Reverses the rotational direction.
    void invertDirection()
    {
        setDirection(!this->reverseDirection);
    }

    // MUST BE CALLED IN LOOP TO ACHIEVE ACCURATE MOVEMENT.
    void updatePos()
    {
        if (targetReached)
            return;

        // Calculate delta T and update the previous time term (previousTime).
        long long int currentTime = micros();
        float deltaT = ((float)(currentTime - previousTime) / 1000000);
        this->previousTime = currentTime;

        // Calculate the error.
        long long int error = this->position - this->target;

        // Calculate the derivative term and update the previous error term (previousError).
        float derivative = (error - this->previousError) / deltaT;
        this->previousError = error;

        // Calculate the integral term.
        this->prevIntegral += error * deltaT;

        // Calculate the PID signal.
        float signal = this->kp * error + this->kd * derivative + this->ki * this->prevIntegral;

        // Calculate speed.
        float speed = fabs(signal);
        if (speed > 255)
        {
            speed = 255;
        }

        // Apply signal.
        setSpeed(speed, signal > 0);

        // Check if the motor has reached its target.
        targetReached = previousError < 1 && previousError > -1;

        // LOGGING...
        Serial.print((int)this->position);
        Serial.print(" ");
        Serial.print((int)this->target);
        Serial.println();
    }

    // Sets a new target distance for the motor to move to.
    void moveTo(float targetDistance)
    {
        this->target = targetDistance * this->distanceFactor;
    }

    // Sets position to 0.
    void reset()
    {
        this->position = 0;
    }
};

// Calibration
const float DISTANCE_FACTOR = 1;                  // Constant of proportionality for converting encoder position ticks to real life distance.
const float RIGHT_ANGLE_ROTATION_DISTANCE = 2000; // Distance that when applied to both motors in opposite directions, turns it 90 degrees.

// Right motor...
const byte R_MOTOR_ENABLE = 6;
const byte R_MOTOR_INPUT1 = 7;
const byte R_MOTOR_INPUT2 = 8;
const byte R_MOTOR_ENCODER_A = 2;
const byte R_MOTOR_ENCODER_B = 3;
Motor rightMotor(R_MOTOR_ENABLE, R_MOTOR_INPUT1, R_MOTOR_INPUT2, R_MOTOR_ENCODER_A, R_MOTOR_ENCODER_B, DISTANCE_FACTOR);

// Left motor...
const byte L_MOTOR_ENABLE = 9;
const byte L_MOTOR_INPUT1 = 10;
const byte L_MOTOR_INPUT2 = 11;
const byte L_MOTOR_ENCODER_A = 12;
const byte L_MOTOR_ENCODER_B = 13;
Motor leftMotor(L_MOTOR_ENABLE, L_MOTOR_INPUT1, L_MOTOR_INPUT2, L_MOTOR_ENCODER_A, L_MOTOR_ENCODER_B, DISTANCE_FACTOR);

void readEncoder(Motor *m)
{
    // Read the second encoder output.
    int b = digitalRead(m->encoderB);

    // If the second output is HIGH, increase position. Otherwise, decrease it.
    b > 0 ? m->position++ : m->position--;
}

void setup()
{
    // Begin serial communication.
    Serial.begin(9600);

    // Attach interrupts...
    attachInterrupt(digitalPinToInterrupt(rightMotor.encoderA), []()
                    { readEncoder(&rightMotor); }, RISING);
    // attachInterrupt(digitalPinToInterrupt(leftMotor.encoderA), []()
    //                 { readEncoder(&leftMotor); }, RISING);

    // TESTING...
    // move(5000);
    // turnLeft();
    turnRight();
}

void turnRight()
{
    rightMotor.moveTo(-RIGHT_ANGLE_ROTATION_DISTANCE);
    leftMotor.moveTo(RIGHT_ANGLE_ROTATION_DISTANCE);
}

void turnLeft()
{
    rightMotor.moveTo(RIGHT_ANGLE_ROTATION_DISTANCE);
    leftMotor.moveTo(-RIGHT_ANGLE_ROTATION_DISTANCE);
}

void move(float distance)
{
    rightMotor.moveTo(distance);
    leftMotor.moveTo(distance);
}

void loop()
{
    rightMotor.updatePos();

    // UNCOMMENT WHEN LEFT MOTOR IS CONNECTED.
    // leftMotor.updatePos();
}
