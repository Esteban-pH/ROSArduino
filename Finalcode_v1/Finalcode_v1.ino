#define USE_USBCON 
#include <ros.h>  
#include <std_msgs/Int32MultiArray.h>
#include <Encoder.h> 

//Ports for the encoder are defined
#define pin_enc_A_motor1 53
#define pin_enc_B_motor1 52
#define pin_enc_A_motor2 51
#define pin_enc_B_motor2 50
#define pin_enc_A_motor3 49
#define pin_enc_B_motor3 48
#define pin_enc_A_motor4 47
#define pin_enc_B_motor4 46
#define MULTIARRAY_SIZE 4 

//Global variables 
ros::NodeHandle nh;
std_msgs::Int32MultiArray encoder_counter; //To keep the count of ticks in the encoder 
ros::Publisher encoder_counter_pub("robot_vel", &encoder_counter); //The ROS publisher to the robot_vel topic 

unsigned long intervalms=10; 

//The encoder best performance is achieved when using interrupts 
Encoder myEnc_1(pin_enc_A_motor1,pin_enc_B_motor1);
Encoder myEnc_2(pin_enc_A_motor2,pin_enc_B_motor2);
Encoder myEnc_3(pin_enc_A_motor3,pin_enc_B_motor3);
Encoder myEnc_4(pin_enc_A_motor4,pin_enc_B_motor4);

void setup() { 
  nh.initNode(); //inits the node 
  encoder_counter.layout.dim = (std_msgs::MultiArrayDimension *) 
  malloc(sizeof(std_msgs::MultiArrayDimension)*MULTIARRAY_SIZE); 
  encoder_counter.layout.dim[0].label = "Encoder"; 
  encoder_counter.layout.dim[0].size = MULTIARRAY_SIZE; 
  encoder_counter.layout.dim[0].stride = 1; 
  encoder_counter.layout.data_offset = 0; 
  //long int is neccesary in case you want to use int32 array
  encoder_counter.data = (long int *)malloc(sizeof(long int)*8); 
  encoder_counter.data_length = MULTIARRAY_SIZE; 
  nh.advertise(encoder_counter_pub); //Inits the publisher
} 

long oldPosition_1  = -999;   
long oldPosition_2  = -999;
long oldPosition_3  = -999;
long oldPosition_4  = -999;

long newPosition_1;
long newPosition_2;
long newPosition_3;
long newPosition_4;


void loop() { 
  //Reads all the encoders
  newPosition_1 = myEnc_1.read();
  newPosition_2 = myEnc_2.read();
  newPosition_3 = myEnc_3.read();
  newPosition_4 = myEnc_4.read();

  //Saves the value of the encoders
  oldPosition_1 = newPosition_1;
  oldPosition_2 = newPosition_2;
  oldPosition_3 = newPosition_3;
  oldPosition_4 = newPosition_4; 

  //Saves the new value of the encoders in an array
  encoder_counter.data[0]=newPosition_1;
  encoder_counter.data[1]=newPosition_2;
  encoder_counter.data[2]=newPosition_3;
  encoder_counter.data[3]=newPosition_4;

  //Publishes the new values in the encoders
  encoder_counter_pub.publish(&encoder_counter); 
  nh.spinOnce(); 
  delay(intervalms); 
}
