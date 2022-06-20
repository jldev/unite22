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

int distance;
int yan = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(dirAPin, OUTPUT);   // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);   // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);   // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);   // set pwmBPin to output mode 
}


#define SPEED 120

void loop() {
   digitalWrite(dirAPin, HIGH);
   analogWrite(pwmAPin, 255);  
   digitalWrite(dirBPin, LOW);   
   analogWrite(pwmBPin, 255);
   delay(100);
   
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
