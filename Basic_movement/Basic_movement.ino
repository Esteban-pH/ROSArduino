#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>

//Driver 1
//Motor 1
int EN11  = 2;
int EN12  = 3;
//Motor 2 
int EN13  = 4;    
int EN14  = 5;    

//Driver 2
//Motor 3
int EN21  = 6;
int EN22  = 7; 
//Motor 4
int EN23  = 8;    
int EN24  = 9;  

//Buttons
int Forward   = 23;
int Backward  = 24;
int TurnRight = 25;
int TurnLeft  = 26;

ros::NodeHandle nh;
double maxrads     = 16.54572131; //Max rads of the wheels, this to compute the pwm
double radio_in_m  = .120; //Input the radio of the wheels in meters
double dis_wheels  = .29;  //Distance between the wheels in meters
double wr, wi, pwm_wr, pwm_wi;

void messageCb(const geometry_msgs::Twist& cmd_vel){
  //Here the speeds for the wheels are calculated based on the given twist
  wr=((2*cmd_vel.linear.x)+cmd_vel.angular.z*dis_wheels)/(2.0*radio_in_m);
  wi=((2*cmd_vel.linear.x)-cmd_vel.angular.z*dis_wheels)/(2.0*radio_in_m);
  //The angular velocity is converted to pwm
  pwm_wr=(wr*255.0)/maxrads;
  pwm_wi=(wi*255.0)/maxrads;

  //Depending on the value the value is written to the wheels, both wheels on each side are treated as one
  if(pwm_wr>0){
    analogWrite(EN11, pwm_wr);
    analogWrite(EN12, 0);
    analogWrite(EN13, pwm_wr);
    analogWrite(EN14, 0);
  }
  else{
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
  }
  else{
    analogWrite(EN21, 0);
    analogWrite(EN22, -pwm_wi);
    analogWrite(EN23, 0);
    analogWrite(EN24, -pwm_wi);
  }
    
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb );

void setup(){
 //Driver 1
 //Motor 1
 pinMode (EN11, OUTPUT); 
 pinMode (EN12, OUTPUT);
 //Motor 2
 pinMode (EN13, OUTPUT);
 pinMode (EN14, OUTPUT);

 //Driver 2
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

 nh.initNode();
 nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
