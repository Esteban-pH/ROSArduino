#define  USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <Encoder.h> 

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
#define pin_enc_A_motor3 49
#define pin_enc_B_motor3 48
#define pin_enc_A_motor4 47
#define pin_enc_B_motor4 46
//This is defined as 4 since it's what is required to send the info from the 4 encoders
#define MULTIARRAY_SIZE 4 

//Buttons
int Forward   = 23;
int Backward  = 24;
int TurnRight = 25;
int TurnLeft  = 26;

//Variables to send the speed
double maxrads     = 16.3362818; //Max rads of the wheels, this to compute the pwm at 12V, to calculate this is maxpwm*2pi/60
//double maxrads     = 8.16814089; //Max rads of the wheels, this to compute the pwm at 6V, to calculate this is maxpwm*2pi/60
double radio_in_m  = .120; //Input the radio of the wheels in meters
double dis_wheels  = .29;  //Distance between the wheels in meters
double wr, wi, pwm_wr, pwm_wi;

//Variables for the encoders and its position
long oldPosition_1  = -999;   
long oldPosition_2  = -999;
long oldPosition_3  = -999;
long oldPosition_4  = -999;

long newPosition_1;
long newPosition_2;
long newPosition_3;
long newPosition_4;

unsigned long intervalms=10; //Frequency of the program

//The encoder best performance is achieved when using interrupts 
Encoder myEnc_1(pin_enc_A_motor1,pin_enc_B_motor1);
Encoder myEnc_2(pin_enc_A_motor2,pin_enc_B_motor2);
Encoder myEnc_3(pin_enc_A_motor3,pin_enc_B_motor3);
Encoder myEnc_4(pin_enc_A_motor4,pin_enc_B_motor4);

void messageCb(const geometry_msgs::Twist& cmd_vel){
  //Here the speeds for the wheels are calculated based on the given twist
  wr=((2*cmd_vel.linear.x)+cmd_vel.angular.z*dis_wheels)/(2.0*radio_in_m);
  wi=((2*cmd_vel.linear.x)-cmd_vel.angular.z*dis_wheels)/(2.0*radio_in_m);
  //The angular velocity is converted to pwm
  pwm_wr=(wr*255.0)/maxrads;
  pwm_wi=(wi*255.0)/maxrads;

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
  digitalWrite(EN11, HIGH);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, HIGH);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, HIGH);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, HIGH);
}

//ROS variables to handle both suscribers and publisher
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
std_msgs::Int32MultiArray encoder_counter; //To keep the count of ticks in the encoder 
ros::Publisher encoder_counter_pub("robot_vel", &encoder_counter); //The ROS publisher to the robot_vel topic

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

 nh.initNode();
 //Initializes the array of integers
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
 nh.subscribe(sub);
}

void loop(){
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

  //Detects when the buttons for manual mode are being pressed
  if(!digitalRead(Forward))
    Forward_function();
  else if(!digitalRead(Backward))
    Backward_function();
  else if(!digitalRead(TurnRight))
    TurnRight_function();
  else if(!digitalRead(TurnLeft))
    TurnLeft_function();
  else
    Stop_function();
  //Publishes the new values in the encoders
  encoder_counter_pub.publish(&encoder_counter); 
  nh.spinOnce(); 
  delay(intervalms); 
}
