#define  USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h> 
#include <PID_v1.h>

//Driver 1, WR
//Motor 1
int EN11  = 2;
int EN12  = 3;
//Motor 2
int EN13  = 4;    
int EN14  = 5;    

//Driver 2, WI
//Motor 3
int EN21  = 6;
int EN22  = 7; 
//Motor 4
int EN23  = 8;    
int EN24  = 9;

//Ports for the encoder are defined
#define pin_enc_A_motor1 53 
#define pin_enc_B_motor1 52
#define pin_enc_A_motor2 51
#define pin_enc_B_motor2 50
#define pin_enc_A_motor3 48
#define pin_enc_B_motor3 49
#define pin_enc_A_motor4 46
#define pin_enc_B_motor4 47

//Buttons
int Forward   = 23;
int Backward  = 25;
int TurnRight = 27;
int TurnLeft  = 29;

//Variables to send the speed
double maxrads     = 6.0; //Max rads of the wheels, this to compute the pwm at 6V, to calculate this is maxpwm*2pi/60
double radio_in_m  = .120; //Input the radio of the wheels in meters
double dis_wheels  = .29;  //Distance between the wheels in meters
double countrev    = 4480.0;
double wr=0, wi=0, pwm_wr=0, pwm_wi=0;

//Variables for the encoders and its position
long oldPosition_1  = -999;   
long oldPosition_2  = -999;
long oldPosition_3  = -999;
long oldPosition_4  = -999;

long newPosition_1;
long newPosition_2;
long newPosition_3;
long newPosition_4;

//Variables Motor 1
double deltapos_1, rads_1;
//Variables Motor 2
double deltapos_2, rads_2;
//Variables Motor 3
double deltapos_3, rads_3;
//Variables Motor 4
double deltapos_4, rads_4;
//Varibles calcular el twist real
double real_wr, real_wi;
//Variables PID controller
double pid_wr, pid_wi;
//PID controllers, it's Input, Output, Setpoing, Kp, Ki, Kd, DIRECT
PID PID_wr(&real_wr, &pid_wr, &wr,1,2,0,DIRECT);
PID PID_wi(&real_wi, &pid_wi, &wi,1,2,0,DIRECT);

unsigned long intervalms=10; //Frequency of the program

//The encoder best performance is achieved when using interrupts 
Encoder myEnc_1(pin_enc_A_motor1,pin_enc_B_motor1);
Encoder myEnc_2(pin_enc_A_motor2,pin_enc_B_motor2);
Encoder myEnc_3(pin_enc_A_motor3,pin_enc_B_motor3);
Encoder myEnc_4(pin_enc_A_motor4,pin_enc_B_motor4);


void messageCb(const geometry_msgs::Twist& cmd_vel){
  //Here the speeds for the wheels are calculated based on the given twist
  wr=((2*cmd_vel.linear.x)+(cmd_vel.angular.z*dis_wheels))/(2.0*radio_in_m);
  wi=((2*cmd_vel.linear.x)-(cmd_vel.angular.z*dis_wheels))/(2.0*radio_in_m);
}

//Functions to manual mode
void Forward_function(){
  digitalWrite(EN11, HIGH);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, HIGH);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, HIGH);
  digitalWrite(EN22, LOW);
  digitalWrite(EN23, HIGH);
  digitalWrite(EN24, LOW);
}

void Backward_function(){
  digitalWrite(EN11, LOW);
  digitalWrite(EN12, HIGH);
  digitalWrite(EN13, LOW);
  digitalWrite(EN14, HIGH);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, HIGH);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, HIGH);
}

void TurnRight_function(){
  digitalWrite(EN11, LOW);
  digitalWrite(EN12, HIGH);
  digitalWrite(EN13, LOW);
  digitalWrite(EN14, HIGH);
  digitalWrite(EN21, HIGH);
  digitalWrite(EN22, LOW);
  digitalWrite(EN23, HIGH);
  digitalWrite(EN24, LOW);
}

void TurnLeft_function(){
  digitalWrite(EN11, HIGH);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, HIGH);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, HIGH);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, HIGH);
}

void Stop_function(){
  digitalWrite(EN11, LOW);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, LOW);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, LOW);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, LOW);
}

//ROS variables to handle both suscribers and publisher
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
geometry_msgs::Twist real_twist;                         //Publish the real twist
ros::Publisher real_twist_pub("robot_vel", &real_twist); //The ROS publisher to the robot_vel topic

void setup(){
 //Driver 1, WR
 //Motor 1
 pinMode (EN11, OUTPUT); 
 pinMode (EN12, OUTPUT);
 //Motor 2
 pinMode (EN13, OUTPUT);
 pinMode (EN14, OUTPUT);

 //Driver 2, WI
 //Motor 3
 pinMode (EN21, OUTPUT); 
 pinMode (EN22, OUTPUT);
 //Motor 4
 pinMode (EN23, OUTPUT);
 pinMode (EN24, OUTPUT);

 //Buttons
 pinMode (Forward,  INPUT_PULLUP);
 pinMode (Backward, INPUT_PULLUP);
 pinMode (TurnRight,INPUT_PULLUP);
 pinMode (TurnLeft, INPUT_PULLUP);

 //Starts ROS nodes
 nh.initNode();
 nh.advertise(real_twist_pub);
 nh.subscribe(sub);

 //Starts PID Controllers
 PID_wr.SetMode(AUTOMATIC);
 PID_wi.SetMode(AUTOMATIC);
 //Default time is 200ms, which can be very slow for many applications
 PID_wr.SetSampleTime(intervalms);
 PID_wi.SetSampleTime(intervalms);
 //This is needed since it starts with 0 to 255 as limits by default
 PID_wi.SetOutputLimits(-maxrads, maxrads);
 PID_wr.SetOutputLimits(-maxrads, maxrads);
}

void loop(){
  //Reads all the encoders
  newPosition_1 = myEnc_1.read();
  newPosition_2 = myEnc_2.read();
  newPosition_3 = myEnc_3.read();
  newPosition_4 = myEnc_4.read();

  //Speed motor 1
  deltapos_1 = (newPosition_1 - oldPosition_1)/countrev;
  rads_1     = (deltapos_1/(intervalms/1000.0))*2*PI;
  //Speed motor 2
  deltapos_2 = (newPosition_2 - oldPosition_2)/countrev;
  rads_2     = (deltapos_2/(intervalms/1000.0))*2*PI;
  //Speed motor 3
  deltapos_3 = (newPosition_3 - oldPosition_3)/countrev;
  rads_3     = (deltapos_3/(intervalms/1000.0))*2*PI;
  //Speed motor 4
  deltapos_4 = (newPosition_4 - oldPosition_4)/countrev;
  rads_4     = (deltapos_4/(intervalms/1000.0))*2*PI;

  real_wr = (rads_1 + rads_2)/2.0;
  real_wi = (rads_3 + rads_4)/2.0;

  //Calculates Twist for debuggin
  real_twist.linear.x  = (radio_in_m*(real_wr+real_wi))/2.0;
  real_twist.angular.z = (radio_in_m*(real_wr-real_wi))/dis_wheels;

  //PID Controller calculation
  PID_wr.Compute();
  PID_wi.Compute();
  
  //The angular velocity is converted to pwm
  pwm_wr=(pid_wr*255.0)/maxrads;
  pwm_wi=(pid_wi*255.0)/maxrads;

  //Depending on the value the value is written to the wheels, both wheels on each side are treated as one
  //If the pwm is positive means forward if it's negative means backwards, then changes the sign
  if(pwm_wr>0){
    analogWrite(EN11, pwm_wr);
    analogWrite(EN12, 0);
    analogWrite(EN13, pwm_wr);
    analogWrite(EN14, 0);
  }else{
    analogWrite(EN11, 0);
    analogWrite(EN12, -pwm_wr);
    analogWrite(EN13, 0);
    analogWrite(EN14, -pwm_wr);
  }

  if(pwm_wi>0){
    analogWrite(EN21, pwm_wi);
    analogWrite(EN22, 0);
    analogWrite(EN23, pwm_wi);
    analogWrite(EN24, 0);
  }else{
    analogWrite(EN21, 0);
    analogWrite(EN22, -pwm_wi);
    analogWrite(EN23, 0);
    analogWrite(EN24, -pwm_wi);
  }
  
  //Saves the value of the encoders
  oldPosition_1 = newPosition_1;
  oldPosition_2 = newPosition_2;
  oldPosition_3 = newPosition_3;
  oldPosition_4 = newPosition_4; 

  //Detects when the buttons for manual mode are being pressed
  if(!digitalRead(Forward))
    Forward_function();
  else if(!digitalRead(Backward))
    Backward_function();
  else if(!digitalRead(TurnRight))
    TurnRight_function();
  else if(!digitalRead(TurnLeft))
    TurnLeft_function();
  else if(pwm_wr==0 && pwm_wi == 0 && digitalRead(TurnLeft) && digitalRead(TurnRight) && digitalRead(Backward) && digitalRead(Forward)){
    Stop_function();
  }

  //Publishes the new values in the encoders
  real_twist_pub.publish(&real_twist); 
  nh.spinOnce(); 
  delay(intervalms); 
}
