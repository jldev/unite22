/***********************************************************
File name:  wifi_arduinoTCP18.ino
Website: www.adeept.com
E-mail: support@adeept.com
Author: Tom
Date: 2019/10/30 
***********************************************************/
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            8
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      3
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Servo ultrasonicServo;           // define servo to control turning of ultrasonic sensor
int trigPin = 10;                  // define Trig pin for ultrasonic ranging module
int echoPin = 11;                  // define Echo pin for ultrasonic ranging module
float maxDistance = 200;          // define the range(cm) for ultrasonic ranging module, Maximum sensor distance is rated at 400-500cm.
float soundVelocity = 340;        // Sound velocity = 340 m/s
float rangingTimeOut = 2 * maxDistance / 100 / soundVelocity * 1000000; // define the timeout(ms) for ultrasonic ranging module
//Motor control port
const int dirAPin = 7;    // define pin used to control rotational direction of motor A
const int pwmAPin = 6;    // define pin for PWM used to control rotational speed of motor A
const int dirBPin = 4;    // define pin used to control rotational direction of motor B
const int pwmBPin = 5;    // define pin for PWM used to control rotational speed of motor B
//Define motor control direction
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 4

#define RIGHT_SENSOR A0
#define MIDDLE_SENSOR A1
#define LEFT_SENSOR A2

int distance;
int yan = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(dirAPin, OUTPUT);   // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);   // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);   // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);   // set pwmBPin to output mode 
  ctrlCar(STOP,0);//A0, A1, A2 tracking does not detect black lines
  ultrasonicServo.attach(12);  // attaches the servo on ultrasonicPin to the servo object
  pinMode(trigPin, OUTPUT); // set trigPin to output mode
  pinMode(echoPin, INPUT);  // set echoPin to input mode
  ultrasonicServo.write(90);
  pinMode(13, OUTPUT);   // set dirAPin to output mode
  for(int i=0;i<3;i++){
      pixels.setPixelColor(i,pixels.Color(0,0,0));
      pixels.show();    
      delay(10);       
  } 
  Serial.begin(9600);
}

int digitalValue = 0;

void loop() {
  digitalValue = analogRead(LEFT_SENSOR);
  Serial.print("Left = ");
  Serial.print(digitalValue); 
  digitalValue = analogRead(MIDDLE_SENSOR);
  Serial.print(" Middle = ");
  Serial.print(digitalValue); 
  digitalValue = analogRead(RIGHT_SENSOR);
  Serial.print(" Right = ");
  Serial.println(digitalValue); 
  delay(250);
}

//Control motor motion direction and speed function
void ctrlCar( int motorDir, byte motorSpd) {
  switch(motorDir){
    case 1:digitalWrite(dirAPin, HIGH);
           digitalWrite(dirBPin, LOW);
           break;
    case 2:digitalWrite(dirAPin, LOW);
           digitalWrite(dirBPin, HIGH);
           break;
    case 3:digitalWrite(dirAPin, HIGH);
           digitalWrite(dirBPin, HIGH);
           break;
    case 4:digitalWrite(dirAPin, LOW);
           digitalWrite(dirBPin, LOW);
           break;    
    default:digitalWrite(dirAPin, LOW);
           digitalWrite(dirBPin, LOW);
           break;     
  }
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
}

float getDistance() {
  unsigned long pingTime; // save the high level time returned by ultrasonic ranging module
  float distance;         // save the distance away from obstacle
  // set the trigPin output 10us high level to make the ultrasonic ranging module start to measure
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // get the high level time returned by ultrasonic ranging module
  pingTime = pulseIn(echoPin, HIGH, rangingTimeOut);
  if (pingTime != 0) {  // if the measure is not overtime
    distance = pingTime * soundVelocity / 2 / 10000;  // calculate the obstacle distance(cm) according to the time of high level returned
    return distance;    // return distance(cm)
  }
  else                  // if the measure is overtime
    return maxDistance; // returns the maximum distance(cm)
}

void Tracking(){
    if(analogRead(A0)>1000&&analogRead(A1)>1000&&analogRead(A2)>1000){
      ctrlCar(FORWARD,120);//Three tracking detections are black lines
    }
    if(analogRead(A0)>1000&&analogRead(A1)<100&&analogRead(A2)<100){
      ctrlCar(RIGHT,120);//Right A2 tracking detection is black line
    }
    if(analogRead(A0)<100&&analogRead(A1)>1000&&analogRead(A2)<100){
      ctrlCar(FORWARD,120);//The middle side A1 tracking is detected as a black line
    }
    if(analogRead(A0)<100&&analogRead(A1)<100&&analogRead(A2)>1000){
      ctrlCar(LEFT,120);//Left A0 tracking detection is black line
    }
    if(analogRead(A0)<100&&analogRead(A1)<100&&analogRead(A2)<100){
      ctrlCar(STOP,0);//A0, A1, A2 tracking does not detect black lines
    }
}
