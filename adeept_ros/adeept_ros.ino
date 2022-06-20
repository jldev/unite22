/***********************************************************
File name:  adeept_ros.ino
Website: 
E-mail: joshua.d.lunn@gmail.com
Author: Josh
Date: 2022/6/18
Desc: Adapted from the Adeepts examples and https://yoraish.com/2021/09/08/a-full-autonomous-stack-a-tutorial-ros-raspberry-pi-arduino-slam/
https://github.com/yoraish/lidar_bot
***********************************************************/
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

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
const int right_dir_pin  = 7;    // define pin used to control rotational direction of motor A
const int right_pwm_pin  = 6;    // define pin for PWM used to control rotational speed of motor A
const int left_dir_pin  = 4;    // define pin used to control rotational direction of motor B
const int left_pwm_pin  = 5;    // define pin for PWM used to control rotational speed of motor B
const bool left_fwd = false;
const bool right_fwd = true;
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


// Default_speed.
const int default_vel = 120;
int state_vel = default_vel;
const int max_vel = 255;

ros::NodeHandle  nh;

void MoveStop() {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
}

void cmd_vel_cb(const geometry_msgs::Twist &msg){
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;
  // Flipped r and l. Added steering scaler.
  float right_cmd = (-z_rotation*1.8)/2.0 + x;
  float left_cmd = 2.0*x - right_cmd;
  bool right_dir = (right_cmd>0)? right_fwd : !right_fwd;
  bool left_dir = (left_cmd>0)? left_fwd : !left_fwd;

  digitalWrite(right_dir_pin, right_dir);
  digitalWrite(left_dir_pin, left_dir);
  
  int right_write = int( default_vel * right_cmd);
  int left_write = int( default_vel * left_cmd );
 
  if (x == 0 && z_rotation == 0){
      MoveStop();
  }
  
  // Advertise the arduino command.
  int abs_left_write =  abs(left_write);
  int abs_right_write = abs(right_write);

  analogWrite(right_pwm_pin, abs_right_write);
  analogWrite(left_pwm_pin,  abs_left_write);
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {
  pinMode(right_pwm_pin, OUTPUT); 
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  // Set initial values for directions. Set both to forward.
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  
  MoveStop();
  
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
  nh.initNode();
  nh.subscribe(sub);
}


void loop() {
  nh.spinOnce();
  delay(1);
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
