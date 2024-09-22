#include <BluetoothSerial.h>



enum CurrentState
{
  INIT,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_LEFT,
  MOVE_RIGHT,
  STOP_S
};
volatile CurrentState currentState = INIT;

#define MAX_DISTANCE 160
#define ULTRASONIC_TIMEOUT (MAX_DISTANCE*58L)

BluetoothSerial SerialBT;
enum Command
{
  STOP = '0',
  START = '1'
};
volatile char command=STOP;
volatile char prevState=STOP;



//__________________________ Define Ultrasonics Pins __________________________//
#define TRIG_FRONT_LEFT   14
#define ECHO_FRONT_LEFT   27

#define TRIG_FRONT_RIGHT   17
#define ECHO_FRONT_RIGHT   5

#define TRIG_RIGHT 25
#define ECHO_RIGHT 26

#define TRIG_LEFT 12
#define ECHO_LEFT 13

#define TRIG_BACK 16
#define ECHO_BACK 4

//__________________________ Define Motors Pins __________________________//
#define LEFT_MOTOR_P1 32
#define LEFT_MOTOR_P2 33
#define RIGHT_MOTOR_P1 18
#define RIGHT_MOTOR_P2 19


//__________________________ Define constants __________________________//
#define SPEED 215
#define minDistance 30
#define TURN_SPEED 110
#define numReadings 3

//__________________________ Ultrasonic Class __________________________//
class Ultrasonic
{
private:
    int _trigPin;
    int _echoPin;
public:
    /// \brief Constructor for the Ultrasonic class.
    /// This constructor sets the trigger and echo pins for the ultrasonic sensor and sets them as an output and input pin respectively.
    /// \param trigPin The number of the trigger pin.
    /// \param echoPin The number of the echo pin.
    Ultrasonic(int trigPin, int echoPin)
    {
        _trigPin = trigPin;
        _echoPin = echoPin;
      
        pinMode(_trigPin, OUTPUT);
        pinMode(_echoPin, INPUT);
    }
    /// \brief Reads the distance from the ultrasonic sensor.
    /// This function reads the distance from the ultrasonic sensor by sending a pulse on the trigger pin, and then reads the echo pin to determine the time of flight of the pulse.
    /// It then calculates the distance from the sensor using the speed of sound and the time of flight.
    /// \return The distance from the sensor in centimeters.
    float read()
    {
        digitalWrite(_trigPin, LOW);
        delayMicroseconds(2);
        
        digitalWrite(_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(_trigPin, LOW);
      
        long duration = pulseIn(_echoPin, HIGH, 10000);
      
        float distance = duration * 0.034 / 2;
      
        return distance;
    }
};

//__________________________ Motor Class __________________________//
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
        if(speed > 0)
        {
        analogWrite(motorPin1, speed);
        analogWrite(motorPin2, 0);
        }
        else if(speed < 0)
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

//__________________________ Motor object __________________________//
Motor motor(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

//__________________________ IR Sensor Class __________________________//
class IRSensor
{
private:
    int _irPin;
public:
    /// \brief Constructor for the IRSensor class.
    /// This constructor sets the pin for the IR sensor and sets it as an input pin.
    /// \param irPin The number of the pin for the IR sensor.
    IRSensor(int irPin)
    {
        _irPin = irPin;
        pinMode(_irPin, INPUT);
    }


    /// \brief Detects if the IR sensor is on a white line.
    /// This function reads the state of the IR sensor and returns true if it is on a white line, and false otherwise.
    /// \return true if the IR sensor is on a white line, false otherwise.
    bool detectWhiteLine()
    {
        return digitalRead(_irPin) == LOW; // Assuming LOW means white line detected
    }
};

//__________________________ Ultrasonic sensor positions __________________________//
enum SensorPosition_t
{
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK,
  LEFT,
  RIGHT
};



//__________________________ Ultrasonic sensor objects __________________________//
Ultrasonic frontLeft(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
Ultrasonic frontRight(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
Ultrasonic back(TRIG_BACK, ECHO_BACK);
Ultrasonic left(TRIG_LEFT, ECHO_LEFT);
Ultrasonic right(TRIG_RIGHT, ECHO_RIGHT);


float measureDistance(SensorPosition_t SensorPosition);

/**
 * \brief Initialises the robot's components.
 * This function initialises the ESP32's serial communication with the Bluetooth module and the serial monitor.
 * It also sets up the IR sensors to trigger interrupts when a white line is detected.
 */
void setup()
{
  //SerialBT.begin("ESP32_Robot");
  //Serial.begin(115200);
}



/**
 * \brief The main loop of the robot.
 * This function implements the main loop of the robot that searches for the opponent robot to attack it.
 * First, it waits for the start command from the Bluetooth module, then it waits for five seconds.
 * After that, it starts moving and using its ultrasonic sensors to detect the opponent robot.
 * If it detects the opponent robot, it turns towards it and moves forward until it is close enough to attack.
 * If it does not detect the opponent robot, it continues moving and searching for it.
 */
void loop()
{
  
  float frontLeftDistance     =   measureDistance(FRONT_LEFT);
  delay(10);
  float frontRightDistance     =   measureDistance(FRONT_RIGHT);
  delay(10);
  float backDistance         =   measureDistance(BACK);
  delayMicroseconds(10);
  float leftDistance         =   measureDistance(LEFT);
  delayMicroseconds(10);
  float rightDistance        =   measureDistance(RIGHT);
  delayMicroseconds(10);

  /*Serial.print("frontLeftDistance");
  Serial.println(frontLeftDistance);

  Serial.print("frontRightDistance");
  Serial.println(frontRightDistance);

  Serial.print("backDistance");
  Serial.println(backDistance);

  Serial.print("leftDistance");
  Serial.println(leftDistance);

  Serial.print("rightDistance");
  Serial.println(rightDistance);
  Serial.println();
  Serial.println();*/

  if(frontLeftDistance < minDistance && frontRightDistance < minDistance)
  {
    //Serial.println("moveForward");
    if(currentState != MOVE_FORWARD)
    {
      motor.moveForward(SPEED);
      currentState = MOVE_FORWARD;
    }
    delayMicroseconds(10);
  }
  else if(leftDistance < minDistance || (frontLeftDistance < minDistance && frontRightDistance > minDistance))
  {
    if(currentState != MOVE_LEFT)
    {
      motor.turnLeft(TURN_SPEED);
      currentState = MOVE_LEFT;
    }
  }
  else if(backDistance < minDistance || rightDistance < minDistance || (frontLeftDistance > minDistance && frontRightDistance < minDistance))
  {
    if(currentState != MOVE_LEFT)
    {
      motor.turnRight(TURN_SPEED);
      currentState = MOVE_RIGHT;
    }
  }
  /*
  else if(backDistance < minDistance || rightDistance < minDistance || (frontLeftDistance > minDistance && frontRightDistance < minDistance))
  {
    while(frontLeftDistance > minDistance || frontRightDistance > minDistance)
    {
      motor.turnRight(TURN_SPEED);
      delay(10);
      frontLeftDistance = measureDistance(FRONT_LEFT);
      frontRightDistance = measureDistance(FRONT_RIGHT);
    }
  }
  
  */
  else
  {
    if(currentState != MOVE_LEFT)
    {
      motor.turnRight(TURN_SPEED);
      currentState = MOVE_RIGHT;
    }
  }
  delayMicroseconds(250);
}
/**
 * \brief Reads the distance from the given ultrasonic sensor position.
 * This function reads the distance from the given ultrasonic sensor position by taking multiple readings and averaging them.
 * The number of readings is given by the global variable numReadings.
 * The switch statement is used to determine which ultrasonic sensor to read from, based on the given SensorPosition.
 * A small delay is added between readings for stability.
 * If the given SensorPosition is not valid, -1 is returned.
 * \param SensorPosition The position of the ultrasonic sensor to read from.
 * \return The average distance from the sensor in centimeters.
 */
float measureDistance(SensorPosition_t SensorPosition)
{
  float totalDistance = 0;

  for (int i = 0; i < numReadings; i++) {
    switch (SensorPosition)
    {
      case FRONT_LEFT:
        totalDistance += frontLeft.read();
        break;
      case FRONT_RIGHT:
        totalDistance += frontRight.read();
        break;
      case BACK:
        totalDistance += back.read();
        break;
      case LEFT:
        totalDistance += left.read();
        break;
      case RIGHT:
        totalDistance += right.read();
        break;
      default:
        return -1;
    }
    delayMicroseconds(10);  // Adding a small delay between readings for stability
  }

  return totalDistance / numReadings;  // Return the average distance
}


