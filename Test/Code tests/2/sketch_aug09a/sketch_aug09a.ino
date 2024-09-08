//__________________________ Define Ultrasonics Pins __________________________//
#define TRIG_FRONT       19
#define ECHO_FRONT       21

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
#define IR_FRONT_LEFT_PIN  34
#define IR_FRONT_RIGHT_PIN  35
#define IR_BACK_LEFT_PIN   36
#define IR_BACK_RIGHT_PIN   39

//__________________________ Define constants __________________________//
#define SPEED           255
#define TURN_SPEED      155
#define minDistance     120

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
  FRONT,
  BACK,
  LEFT,
  RIGHT
};

//__________________________ Ultrasonic sensor objects __________________________//
Ultrasonic front(TRIG_FRONT, ECHO_FRONT);
Ultrasonic back(TRIG_BACK, ECHO_BACK);
Ultrasonic left(TRIG_LEFT, ECHO_LEFT);
Ultrasonic right(TRIG_RIGHT, ECHO_RIGHT);

//__________________________ IR sensor objects __________________________//
IRSensor IRFrontLeft(IR_FRONT_LEFT_PIN);
IRSensor IRFrontRight(IR_FRONT_RIGHT_PIN);
IRSensor IRBackLeft(IR_BACK_LEFT_PIN);
IRSensor IRBackRight(IR_BACK_RIGHT_PIN);


//__________________________ Motor object __________________________//
Motor motor(LEFT_MOTOR_P1, LEFT_MOTOR_P2, RIGHT_MOTOR_P1, RIGHT_MOTOR_P2);

float measureDistance(SensorPosition_t SensorPosition)
{
  switch (SensorPosition)
  {
    case FRONT:
      return front.read();
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
void IRFrontLeftISR();
void IRFrontRightISR();
void IRBackLeftISR();
void IRBackRightISR();

void setup()
{
  Serial.begin(9600);
  
  attachInterrupt(digitalPinToInterrupt(IR_FRONT_LEFT_PIN), IRFrontLeftISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_FRONT_RIGHT_PIN), IRFrontRightISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_BACK_LEFT_PIN), IRBackLeftISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_BACK_RIGHT_PIN), IRBackRightISR, FALLING);
}

void loop()
{
  float frontDistance     =   measureDistance(FRONT);
  delay(10);
  float backDistance         =   measureDistance(BACK);
  delay(10);
  float leftDistance         =   measureDistance(LEFT);
  delay(10);
  float rightDistance        =   measureDistance(RIGHT);
  delay(10);

  if(frontDistance < minDistance)
  {
    motor.moveForward(SPEED);
    delay(10);
  }
  
  else if(leftDistance < minDistance)
  {
    while(frontDistance > minDistance)
    {
      motor.turnLeft(TURN_SPEED);
      delay(10);
      frontDistance = measureDistance(FRONT);
    }
  }

  else if(backDistance < minDistance || rightDistance < minDistance)
  {
    while(frontDistance > minDistance)
    {
      motor.turnRight(TURN_SPEED);
      delay(10);
      frontDistance = measureDistance(FRONT);
    }
  }
  else
  {
    while(frontDistance > minDistance)
    {
      motor.turnRight(TURN_SPEED);
      delay(10);
      frontDistance = measureDistance(FRONT);
    }
  }
  delay(10);
}


//__________________________ Interrupt Handlers Definition __________________________//
void IRFrontLeftISR()
{
  while(IRFrontLeft.detectWhiteLine())
  {
    
    if(IRFrontRight.detectWhiteLine())
    {
      Serial.println("IRFrontLeftISR");
      Serial.println("IRFrontRightISR");
    }
    else if(IRBackLeft.detectWhiteLine())
    {
      Serial.println("IRFrontLeftISR");
      Serial.println("IRBackLeftISR");
    }
    else
    {
      Serial.println("IRFrontLeftISR");
    }
  }
}

void IRFrontRightISR()
{
  while(IRFrontRight.detectWhiteLine())
  {
    
    if(IRFrontLeft.detectWhiteLine())
    {
      Serial.println("IRFrontRightISR");
      Serial.println("IRFrontLeftISR");
    }
    else if(IRBackRight.detectWhiteLine())
    {
      Serial.println("IRFrontRightISR");
      Serial.println("IRBackRightISR");
    }
    else
    {
      Serial.println("IRFrontRightISR");
    }
  }
}

void IRBackLeftISR()
{
  while(IRBackLeft.detectWhiteLine())
  {
    
    if(IRBackRight.detectWhiteLine())
    {
      Serial.println("IRBackLeftISR");
      Serial.println("IRBackRightISR");
    }
    else if(IRFrontLeft.detectWhiteLine())
    {
      Serial.println("IRBackLeftISR");
      Serial.println("IRFrontLeftISR");
    }
    else
    {
      Serial.println("IRBackLeftISR");
    }
  }
}

void IRBackRightISR()
{
  while(IRBackRight.detectWhiteLine())
  {
    if(IRBackLeft.detectWhiteLine())
    {
      Serial.println("IRBackRightISR");
      Serial.println("IRBackLeftISR");
    }
    else if(IRFrontRight.detectWhiteLine())
    {
      Serial.println("IRBackRightISR");
      Serial.println("IRFrontRightISR");
    }
    else
    {
      Serial.println("IRBackRightISR");
    }
  }
}
