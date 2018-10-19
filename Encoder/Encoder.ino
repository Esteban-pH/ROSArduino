/* Program to read one  encoder connected to an arduino DUE
* This is a simple program that reads an encoder and prints the value to rosserial
* It publishes the data to the encoder_counter topic.
*/
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <Encoder.h>
#define pin_enc_A 53  //Pin of the arduino where the encoder port A is connected
#define pin_enc_B 51  //Pin where the encoder port B is connected
/*
* Global variables
*/
ros::NodeHandle nh;
std_msgs::Int32 encoder_counter; //To keep the count of ticks in the encoder
ros::Publisher encoder_counter_pub("encoder_counter", &encoder_counter); //The ROS publisher to the encoder_counter topic
 
//The encoder best performance is achieved when using interrupts
Encoder myEnc(pin_enc_A,pin_enc_B); //Avoid using pins with LEDs attached
 
 
 
void setup() {
  nh.initNode(); //inits the node
  nh.advertise(encoder_counter_pub); //Inits the publisher
}
 
long oldPosition  = -999;
 
void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    encoder_counter.data = newPosition;
    encoder_counter_pub.publish(&encoder_counter);
    nh.spinOnce();
  }
  
  nh.spinOnce();
  delay(1);
}
