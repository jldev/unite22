#include <Servo.h>
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

#define MOTOR_DEFAULT_SPEED 110

#define BLACK_LINE false

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
  Serial.begin(9600);
}


void loop() {
    int left_reading = readLineSensor(LEFT_SENSOR);
    int middle_reading = readLineSensor(MIDDLE_SENSOR);
    int right_reading = readLineSensor(RIGHT_SENSOR);  

    Serial.print("Left - ");
    Serial.print(left_reading);
    Serial.print("\tMiddle - ");
    Serial.print(middle_reading);
    Serial.print("\tRight - ");
    Serial.println(right_reading);
}

//Control motor motion direction and speed function
void ctrlCar( int motorDir, byte motorSpd) {
  switch(motorDir){
    case FORWARD:digitalWrite(dirAPin, HIGH);
           digitalWrite(dirBPin, LOW);
           break;
    case BACKWARD:digitalWrite(dirAPin, LOW);
           digitalWrite(dirBPin, HIGH);
           break;
    case LEFT:digitalWrite(dirAPin, HIGH);
           digitalWrite(dirBPin, HIGH);
           break;
    case RIGHT:digitalWrite(dirAPin, LOW);
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

boolean onLine(int reading){
  if(BLACK_LINE){
    return reading > 1000;
  } else {
    return reading < 100;
  }
}

int readLineSensor(int pin){
  int return_value = 0;
  do{
    return_value = analogRead(pin);
  } while(return_value == 1023);
  return return_value;
}
