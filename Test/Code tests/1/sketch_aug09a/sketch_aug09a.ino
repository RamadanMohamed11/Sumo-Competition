//__________________________ Define Ultrasonics Pins __________________________//
#define TRIG_FRONT_LEFT  5
#define ECHO_FRONT_LEFT  18

#define TRIG_FRONT_MID   19
#define ECHO_FRONT_MID   21

#define TRIG_FRONT_RIGHT 22
#define ECHO_FRONT_RIGHT 23

#define TRIG_BACK        32
#define ECHO_BACK        33

#define TRIG_LEFT        25
#define ECHO_LEFT        26

#define TRIG_RIGHT       27
#define ECHO_RIGHT       14

//__________________________ Define Motors Pins __________________________//
#define LEFT_MOTOR_P1   2
#define LEFT_MOTOR_P2   4
#define RIGHT_MOTOR_P1 15
#define RIGHT_MOTOR_P2 13

//__________________________ Define IR Sensor Pins __________________________//
#define IR_FRONT_PIN  34
#define IR_BACK_PIN   35
#define IR_LEFT_PIN   36
#define IR_RIGHT_PIN  39

//__________________________ Define constants __________________________//
#define SPEED           255
#define TURN_SPEED      155
#define minDistance     15

//__________________________ Ultrasonic Class __________________________//
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

//__________________________ Motor Class __________________________//
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

//__________________________ IR Sensor Class __________________________//
class IRSensor
{
private:
    int _irPin;
public:
    IRSensor(int irPin)
    {
        _irPin = irPin;
        pinMode(_irPin, INPUT);
    }

    bool detectWhiteLine()
    {
        return digitalRead(_irPin) == LOW; // Assuming LOW means white line detected
    }
};

//__________________________ Ultrasonic sensor positions __________________________//
enum SensorPosition_t
{
  FRONT_LEFT,
  FRONT_MID,
  FRONT_RIGHT,
  BACK,
  LEFT,
  RIGHT
};

//__________________________ Ultrasonic sensor objects __________________________//
Ultrasonic frontLeft(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
Ultrasonic frontMid(TRIG_FRONT_MID, ECHO_FRONT_MID);
Ultrasonic frontRight(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
Ultrasonic back(TRIG_BACK, ECHO_BACK);
Ultrasonic left(TRIG_LEFT, ECHO_LEFT);
Ultrasonic right(TRIG_RIGHT, ECHO_RIGHT);

//__________________________ IR sensor objects __________________________//
IRSensor irFront(IR_FRONT_PIN);
IRSensor irBack(IR_BACK_PIN);
IRSensor irLeft(IR_LEFT_PIN);
IRSensor irRight(IR_RIGHT_PIN);


//__________________________ Motor object __________________________//
Motor motor(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

float measureDistance(SensorPosition_t SensorPosition)
{
  switch (SensorPosition)
  {
    case FRONT_LEFT:
      return frontLeft.read();
    case FRONT_MID:
      return frontMid.read();
    case FRONT_RIGHT:
      return frontRight.read();
    case BACK:
      return back.read();
    case LEFT:
      return left.read();
    case RIGHT:
      return right.read();
    default:
      return -1;
  }
}

//__________________________ Interrupt Handlers Declaration __________________________//
void IRFrontISR();
void IRBackISR();
void IRLeftISR();
void IRRightISR();

void setup()
{
  attachInterrupt(digitalPinToInterrupt(IR_FRONT_PIN), IRFrontISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_BACK_PIN), IRBackISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_LEFT_PIN), IRLeftISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_RIGHT_PIN), IRRightISR, FALLING);
}

void loop()
{
  float frontLeftDistance    =   measureDistance(FRONT_LEFT);
  delay(10);
  float frontMidDistance     =   measureDistance(FRONT_MID);
  delay(10);
  float frontRightDistance   =   measureDistance(FRONT_RIGHT);
  delay(10);
  float backDistance         =   measureDistance(BACK);
  delay(10);
  float leftDistance         =   measureDistance(LEFT);
  delay(10);
  float rightDistance        =   measureDistance(RIGHT);
  delay(10);

  if(frontMidDistance < minDistance)
  {
    motor.moveForward(SPEED);
    delay(10);
  }
  
  else if(frontLeftDistance < minDistance || leftDistance < minDistance)
  {
    
    while(frontMidDistance > minDistance)
    {
      motor.turnLeft(TURN_SPEED);
      delay(10);
      frontMidDistance = measureDistance(FRONT_MID);
    }
  }

  else if(frontRightDistance < minDistance || backDistance < minDistance || rightDistance < minDistance)
  {
    while(frontMidDistance > minDistance)
    {
      motor.turnRight(TURN_SPEED);
      delay(10);
      frontMidDistance = measureDistance(FRONT_MID);
    }
  }
  delay(10);
}


//__________________________ Interrupt Handlers Definition __________________________//

void IRFrontISR()
{
    if (irFront.detectWhiteLine())
    {
        motor.moveBackward(SPEED); // Move backward if white line is detected in front
        delay(500);
        motor.turnRight(TURN_SPEED);
        delay(250);
    }
}

void IRBackISR()
{
    if (irBack.detectWhiteLine())
    {
        motor.moveForward(SPEED); // Move forward if white line is detected in back
        delay(500);
        motor.turnLeft(TURN_SPEED);
        delay(250);
    }
}

void IRLeftISR()
{
    if (irLeft.detectWhiteLine())
    {
        motor.turnRight(TURN_SPEED); // Turn right if white line is detected on the left
        delay(500);
        motor.moveForward(SPEED);
        delay(500);
    }
}

void IRRightISR()
{
    if (irRight.detectWhiteLine())
    {
        motor.turnLeft(TURN_SPEED); // Turn left if white line is detected on the right
        delay(500);
        motor.moveForward(SPEED);
        delay(500);
    }
}
