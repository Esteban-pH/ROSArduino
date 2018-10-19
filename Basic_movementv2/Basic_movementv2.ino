#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>

float r = .056;
float l = .29;
float wr, wl;


//Driver 1
int EN11  = 22;
int EN12  = 23; 
int EN13  = 24;    
int EN14  = 25;    

//Driver 2
int EN21  = 26;
int EN22  = 27; 
int EN23  = 28;    
int EN24  = 29;  

ros::NodeHandle nh;

void messageCb(const geometry_msgs::Twist& cmd_vel){
  if(cmd_vel.linear.x!=0 || cmd_vel.angular.z!=0)
    wr=(2*cmd_vel.linear.x+cmd_vel.angular.z*l)/(2.0*r);
    wl=(2*cmd_vel.linear.x-cmd_vel.angular.z*l)/(2.0*r);
    movement(wr,wl);
  else
    stop();
}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &messageCb );

void setup(){
 //Driver 1
 pinMode (EN11, OUTPUT); 
 pinMode (EN12, OUTPUT);
 pinMode (EN13, OUTPUT);
 pinMode (EN14, OUTPUT);

 //Driver 2
 pinMode (EN21, OUTPUT); 
 pinMode (EN22, OUTPUT);
 pinMode (EN23, OUTPUT);
 pinMode (EN24, OUTPUT);

 nh.initNode();
 nh.subscribe(sub);
}

void movement(float wr, float wl){
  if(wr>0({
    analogWrite(EN11, (int)wr/255);
    analogWrite(EN12, 0);
    analogWrite(EN21, (int)wr/255);
    analogWrite(EN22, 0);
  }else if(wr<0){
    analogWrite(EN11, 0);
    analogWrite(EN12, (int)-wr/255);
    analogWrite(EN21, 0);
    analogWrite(EN22, (int)-wr/255);
  }
  
  if(wl>0){
    analogWrite(EN13, (int)wl/255);
    analogWrite(EN14, 0);
    analogWrite(EN23, (int)wl/255);
    analogWrite(EN24, 0);
  }else if(wl<0){
    analogWrite(EN13, 0);
    analogWrite(EN14, (int)-wl/255);
    analogWrite(EN23, 0);
    analogWrite(EN24, (int)-wl/255);
  }  
}

void stop(){
  digitalWrite(EN11, LOW);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, LOW);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, LOW);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, LOW); 
}

void loop(){
  nh.spinOnce();
  delay(1);
}
