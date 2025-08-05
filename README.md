# Auto-Move-4x-and-Smart-Servo-Reaction-System
(1) Auto Move 4x

https://www.tinkercad.com/things/f4CYPByy1Ch-automove-4x?sharecode=6XIvmRNfPx4VSvr1aT3SiAZq0di_dUKMqYGYYhHYeho

Project Idea:

My project idea is to operate four DC motors using the L293D chip, allowing me to control the direction of each motor (forward or reverse) using the Arduino. I used the chip because it allows me to operate more than one motor at the same time and distribute control among them through simple programming.

Project Goal:

The goal of the project is to learn how to control multiple motors using a single chip, how to connect it to the Arduino, and how to program each motor to move differently or with the others. The goal is to apply this foundation to other projects, such as a smart car or a mobile robot.

Project Benefits:

The project benefits me by showing me how to control multiple motors using the L293D chip instead of needing many parts. It also shows me how to program the Arduino to give precise commands to each motor. The same idea can be expanded and used in many applications, such as:
- Robotics
- Smart cars
-  Robotic arms

Tools used:
- 4xDC motors
- 2xL293D
- Breadboard
- Arduino uno
- 9v battery
- Wires

Project implementation:

First, I connected the four DC motors to the L293D chip. Each motor has two control pins (IN1 and IN2, for example) and an enable pin (EN) so I can control them via the Arduino.

Using the code, I program each motor to:
-  Move forward or backward based on the signals I send from the Arduino.
-  Each motor can be set to start at a specific time, stop, or move in reverse.

The L293D chip connects the Arduino to the motors, helping them deliver power to the motors and receive and execute commands from the Arduino. This way, I can control each motor individually or have them all operate together in a specific pattern, such as: forward for a period, then reverse, or even right and left in rotation.

Code:

int m1_in1 = 2;
int m1_in2 = 3;

int m2_in1 = 4;
int m2_in2 = 5;

int m3_in1 = 6;
int m3_in2 = 7;

int m4_in1 = 8;
int m4_in2 = 9;

void setup() {
 
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(m3_in1, OUTPUT);
  pinMode(m3_in2, OUTPUT);
  pinMode(m4_in1, OUTPUT);
  pinMode(m4_in2, OUTPUT);
  pinMode (10,OUTPUT);
  pinMode(11,OUTPUT);
   
  digitalWrite(10,HIGH);
  digitalWrite(11,HIGH);

 
  moveForward();
  delay(30000);

 
  moveBackward();
  delay(60000);

 
  alternateRightLeft(60000);

 
  stopAllMotors();
}

void loop() 
{
 
}

void moveForward() 
{
  motorForward(m1_in1, m1_in2);
  motorForward(m2_in1, m2_in2);
  motorForward(m3_in1, m3_in2);
  motorForward(m4_in1, m4_in2);
}

void moveBackward() 
{
  motorBackward(m1_in1, m1_in2);
  motorBackward(m2_in1, m2_in2);
  motorBackward(m3_in1, m3_in2);
  motorBackward(m4_in1, m4_in2);
}

void alternateRightLeft(long durationMs) 
{
  long start = millis();
  while (millis() - start < durationMs)
  {
    moveRight();
    delay(1000);
    moveLeft();
    delay(1000);
  }
}

void moveRight() 
{
  motorForward(m1_in1, m1_in2);
  motorForward(m3_in1, m3_in2);
  motorStop(m2_in1, m2_in2);
  motorStop(m4_in1, m4_in2);
}

void moveLeft()
{
  motorForward(m2_in1, m2_in2);
  motorForward(m4_in1, m4_in2);
  motorStop(m1_in1, m1_in2);
  motorStop(m3_in1, m3_in2);
}

void stopAllMotors() 
{
  motorStop(m1_in1, m1_in2);
  motorStop(m2_in1, m2_in2);
  motorStop(m3_in1, m3_in2);
  motorStop(m4_in1, m4_in2);
}



void motorForward(int in1, int in2)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorBackward(int in1, int in2) 
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void motorStop(int in1, int in2) 
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

-----------------------------------------------------------

(2)Smart Servo Reaction System

https://www.tinkercad.com/things/cxPjyFn2B1Q-smart-servo-reaction-system?sharecode=Z_FM34DH-qhdASWoUzfDkTeC9vGg3JJ95NmnPCUYj0E

Project Idea:

My project is an ultrasonic sensor connected to a servo motor. When an object approaches the sensor at a distance of less than 10 cm, it commands the servo to move to a specific angle, as if responding to the approaching object. The servo then returns to its normal position.

Project Goal:

The goal is for me to learn how to use sensors with motors and how to program interactions between them using Arduino.
The project is simple, but it helps me understand the connection between electronics and programming in a practical way.

Project Benefits:

The project benefits me by demonstrating the concept of automatic interaction between sensors and actuators (such as servo motors). The same idea can be developed and used in larger applications, such as:
• Security systems (such as smart doors)
• Robots that interact with surrounding objects
• Or even alarm systems if an object approaches a certain distance

Tools Used:

- Arduino uno 
- Ultrasonic HC-SR04
- Servo Motor
- L293D
- Breadboard 
- Wires

Project Method:

First, I connect the ultrasonic sensor to the Arduino using two terminals:
• Trig sends an audio pulse
• Echo receives the echo

The sensor works by sending an inaudible audio wave. If there is an object in front of it, the wave bounces back. The Arduino calculates the time it takes for the wave to travel and return, and based on this, it calculates the distance between the sensor and the object.

I program the Arduino to trigger a specific reaction if the distance is less than 10 cm—such as:
• Moving a servo motor
• Starting a DC motor
• Turning on a light or sound

Whenever an object approaches the sensor, the device automatically responds, as if it "senses" the obstacle and reacts to it.

Code:

#include <Servo.h>

Servo myServo;
const int servoPin = 9;

const int trigPin = 6;
const int echoPin = 7;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin);
  myServo.write(90);  
}

void loop() {
  long distance = readDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= 10) {
    
    myServo.write(30);   
    delay(500);
    myServo.write(150);  
    delay(500);
    myServo.write(90);   
  }

  delay(200);
}

long readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}
