#include <Servo.h>
#include <NewPing.h>

#define MAX_MOTOR_SPEED 255
//Right motor
int enableRightMotor=5; 
int rightMotorPin1=7;
int rightMotorPin2=8;
//Left motor
int enableLeftMotor=6;
int leftMotorPin1=9;
int leftMotorPin2=11;

#define SERVO_PIN 3
#define ULTRASONIC_SENSOR_TRIG A5
#define ULTRASONIC_SENSOR_ECHO A4

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 200);
Servo myServo;

void setup()
{
  // put your setup code here, to run once:
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);
  
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  rotateMotor(0,0);   
    
  Serial.begin(9600);
}



void loop()
{
  if (Serial.available())
  {
    char inputCommand = Serial.read();
    
    if (inputCommand == 'F')
    {
      rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    }

    if (inputCommand == 'B')
    {
      rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
    }
    
    if (inputCommand == 'L')
    {
      rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
    }

    if (inputCommand == 'R')
    {
      rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    }

    if (inputCommand == 'S')
    {
      rotateMotor(0,0);
    }
  } 

  int distance = mySensor.ping_cm();
  Serial.write((char*)&distance, 2);
  delay(50);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }


  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
