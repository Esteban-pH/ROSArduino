#define USE_USBCON 
#include <ros.h> 
#include <std_msgs/Int32.h>   
#include <Encoder.h> 
#define pin_enc_A 53  //Pin of the arduino where the encoder port A is connected 
#define pin_enc_B 51  //Pin where the encoder port B is connected 


//Global variables 
ros::NodeHandle nh; 
std_msgs::Int32 encoder_counter; //To keep the count of ticks in the encoder 
ros::Publisher encoder_counter_pub("robot_vel", &encoder_counter); //The ROS publisher to the robot_vel topic 
unsigned long stime;
unsigned long intervalms=10; 

//The encoder best performance is achieved when using interrupts 
Encoder myEnc(pin_enc_A,pin_enc_B); //Avoid using pins with LEDs attached 

void setup() { 
  nh.initNode(); //inits the node 
  nh.advertise(encoder_counter_pub); //Inits the publisher
  stime = millis(); 
} 

long oldPosition  = -999;   

void loop() { 
  long newPosition = myEnc.read(); 
  oldPosition = newPosition; 
  encoder_counter.data = newPosition; 
  encoder_counter_pub.publish(&encoder_counter); 
  stime = millis();
  nh.spinOnce(); 
  nh.spinOnce(); 
  delay(intervalms); 
}
