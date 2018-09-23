/*
   rosserial Planar Odometry Example
*/
//#define USE_USBCON              //only for serial connection (in this case we used wifi)
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Range.h>
#include <Wire.h>
#include <QuadratureEncoder.h>
#include <Odometer.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Scheduler.h>
#include <Arduino.h>
#include <SPI.h>
#include <WiFi101.h>

const char ssid[] = "SSID";     // your network SSID (name)
const char pass[] = "PASS";   // your network password


//--------tcp-ip-setup---------------
IPAddress server(192, 168, 1, 2);         //ros masters ip
WiFiClient client;

class WiFiHardware{
  public:
  WiFiHardware(){};
  void init(){
    // TCP server/client setup
    client.connect(server, 11411);
  }
  // read a byte from the serial port. (-1 = failure)
  int read(){
    // implemention of this method so that it reads a byte from the TCP connection and returns it
    return client.read();         
  }
  // write data to the connection to ROS
  void write(uint8_t* data, int length){
    // implemention of this so that it takes the arguments and writes them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }
  // returns milliseconds since start of program
  unsigned long time(){
     return millis();
  }
};

/*
   ------------general variables--
*/
//motor controller pin setup
const int leftForwardPin = 3;
const int leftBackwardPin = 2;
const int rightForwardPin = 0;
const int rightBackwardPin = 1;
const int leftMotorPwm = 18;
const int rightMotorPwm = 19;
//encoder pin setup
const int encoderLA = 4;
const int encoderLB = 5;
const int encoderRA = 7;
const int encoderRB = 6;
//ulrasonic modules pin setup
const int leftUTrigPin = 13;
const int leftUEchoPin = 14;
const int rightUTrigPin = 21;
const int rightUEchoPin = 20;
//bno055 setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> gyro;
const int bnoRstPin = 8;
//localizaton variables
float head = 0;
float xVector = 0;
float yVector = 0;
float dWay = 0;
float ddWay = 0;
float hold = 0;
//object definitions
Encoder leftEncoder(LEFT);
Encoder rightEncoder(RIGHT);
Odometer odom;
// robots physical values
const double radius_of_wheels = 3;        //r(cm)
const double wide_length = 11.2;          //l(cm)
const float motor_one_turn_tick = 700.0;  
const float perimeter_of_wheels = 0.195;  //(m)

/*
    ---------function declerations--
*/
void rightHandlerA();   //functions for encoder interrupt handlers
void rightHandlerB();
void leftHandlerA();
void leftHandlerB();
void cmd_vel_handle(const geometry_msgs::Twist& msg);
void fix_velocities();
float getRangeUltrasound(int trigPin,int echoPin);
float absoluteValue(float value);
float map_func(float value, float inMin, float inMax, float outMin, float outMax);
inline double degToRad(double deg);

/*
   ---------function variable definitions--
*/
//cmd velocity handler
float fwd_message;
float turn_message;
float right_cmd_vel;
float left_cmd_vel;
int mapped_right_vel;
int mapped_left_vel;
//ultrasonic sensor handler
long ultrasonic_range_time;
char ultrasonic_frameid[] = "/gonzales/ultrasonic";
long ultrasonic_duration;
//velocity fixer
long right_enc_pos;
long right_enc_old_pos;
long left_enc_pos;
long left_enc_old_pos;
unsigned long enc_velfix_time_now;
unsigned long enc_velfix_time_old;
float right_enc_vel;
float left_enc_vel;
//pose2d
double x = 1.0;
double y = 0.0;
double theta = 1.57;

/*
    ------------- ROS ------------
*/
//ros::NodeHandle  nh;                  //for serial connections (in this case we used wifi)
ros::NodeHandle_<WiFiHardware> nh;

//pose2d
geometry_msgs::Pose2D pose2d;
ros::Publisher ros_odom("/gonzales/pose2d", &pose2d);

//ultrasonic sensors
sensor_msgs::Range range_msg;
ros::Publisher pub_range1("/gonzales/ultrasonicRight", &range_msg);
ros::Publisher pub_range2("/gonzales/ultrasonicLeft", &range_msg);

//cmd velocity
ros::Subscriber<geometry_msgs::Twist> sub("/gonzales/cmd_vel", &cmd_vel_handle );

/*
    ------------- ARDUINO ------------
*/
void setup(){
  WiFi.begin(ssid, pass); //wifi connection
  delay(1000);
  //rosnode initialize
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(ros_odom);
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  delay(1000);
  
  //ultrasonic sensors setup
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  ultrasonic_frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 2.0;
  range_msg.max_range = 200.0;
  
  //motor encoders setup
  rightEncoder.attach(encoderRA, encoderRB);
  leftEncoder.attach(encoderLA, encoderLB);
  leftEncoder.initialize();
  rightEncoder.initialize();
  attachInterrupt(digitalPinToInterrupt(encoderLA), leftHandlerA , CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), leftHandlerB , CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRA), rightHandlerA , CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRB), rightHandlerB , CHANGE);
  
  //motor controller setup
  pinMode(leftForwardPin,OUTPUT);
  pinMode(leftBackwardPin,OUTPUT);
  pinMode(rightForwardPin,OUTPUT);
  pinMode(rightBackwardPin,OUTPUT);
  pinMode(rightMotorPwm, OUTPUT);
  pinMode(leftMotorPwm, OUTPUT);
  
  //Ultrasonic modules setup
  pinMode(leftUTrigPin, OUTPUT);
  pinMode(leftUEchoPin, INPUT);
  pinMode(rightUTrigPin, OUTPUT);
  pinMode(rightUEchoPin, INPUT);
  
  //reset bno055
  pinMode(bnoRstPin, OUTPUT);
  digitalWrite(bnoRstPin, LOW);
  delay(500);
  digitalWrite(bnoRstPin, HIGH);
  delay(500);
  
  //initialize bno055 absolute orientation sensor
  while (!bno.begin()) {
    nh.logerror("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(100);
  }
  bno.setExtCrystalUse(true);
  delay(100);
  
  // Add "loop2" to scheduling.
  // "loop" is always started by default.
  Scheduler.startLoop(loop2);
  //save encoders first variables as old
  right_enc_old_pos = rightEncoder.encoderTicks;
  left_enc_old_pos = leftEncoder.encoderTicks;
  enc_velfix_time_old = millis();
}

void loop(){
  //get values from encoders for odometer
  hold = odom.getWay();
  dWay = hold - ddWay;
  ddWay = hold;
  sensors_event_t event;
  bno.getEvent(&event);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  yVector = yVector - dWay * sin(degToRad(event.orientation.x));
  xVector = xVector + dWay * cos(degToRad(event.orientation.x));

  //correct the orientation
  pose2d.x = xVector / 10.0;
  pose2d.y = yVector / 10.0;
  pose2d.theta = degToRad(map_func(event.orientation.x , 0 , 360 , 360 , 0));
  ros_odom.publish( &pose2d );
  
  fix_velocities();
  
  nh.spinOnce();
  delay(2);
}

void loop2(){
  //publish the adc value every 50 milliseconds since it takes that long for the ultrasonic sensor to stabilize
  if ( millis() >= ultrasonic_range_time ){
    range_msg.range = getRangeUltrasound(rightUTrigPin,rightUEchoPin);
    range_msg.header.stamp = nh.now();
    pub_range1.publish(&range_msg);

    range_msg.range = getRangeUltrasound(leftUTrigPin,leftUEchoPin);
    range_msg.header.stamp = nh.now();
    pub_range2.publish(&range_msg);

    ultrasonic_range_time =  millis() + 50;
  }
  
  nh.spinOnce();
  delay(2);
  yield();
}

/*
  ------------functions bodies-------
*/
void rightHandlerA() {
  rightEncoder.handleInterruptGreen();
  odom.rightEncoderTick = rightEncoder.encoderTicks;
}
void rightHandlerB() {
  rightEncoder.handleInterruptYellow();
  odom.rightEncoderTick = rightEncoder.encoderTicks;
}
void leftHandlerA() {
  leftEncoder.handleInterruptGreen();
  odom.leftEncoderTick = leftEncoder.encoderTicks;
}
void leftHandlerB() {
  leftEncoder.handleInterruptYellow();
  odom.leftEncoderTick = leftEncoder.encoderTicks;
}

// returns absolute values of mapped velocities because we used negativity for direction setting
float absoluteValue(float value){
  if (value < 0)
    return (value * -1);
  else
    return value;
}

float map_func(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

//reads cmd_vel topic then turns the motors
void cmd_vel_handle( const geometry_msgs::Twist& msg){
  fwd_message = msg.linear.x;
  turn_message = msg.angular.z;
  right_cmd_vel = int(((2.0 * fwd_message)+(turn_message * wide_length)) / (2.0 * radius_of_wheels));
  left_cmd_vel = int(((2.0 * fwd_message)-(turn_message * wide_length)) / (2.0 * radius_of_wheels));
  if ((right_cmd_vel < 0.5) && (right_cmd_vel > -0.5)){
    right_cmd_vel *= 2;
  }
  if ((left_cmd_vel < 0.5)  && (left_cmd_vel > -0.5)){
    left_cmd_vel *= 2;  
  }
  mapped_right_vel = int(map_func(absoluteValue(right_cmd_vel) , 0 , 6 , 0 , 255));
  mapped_left_vel = int(map_func(absoluteValue(left_cmd_vel) , 0 , 6 , 0 , 255));
  
  if (right_cmd_vel >= 0){
    digitalWrite(rightForwardPin,HIGH);
    digitalWrite(rightBackwardPin,LOW);
  }
  else{
    digitalWrite(rightForwardPin,LOW);
    digitalWrite(rightBackwardPin,HIGH);
  }
  if (left_cmd_vel >= 0){
    digitalWrite(leftForwardPin,HIGH);
    digitalWrite(leftBackwardPin,LOW);
  }
  else{
    digitalWrite(leftForwardPin,LOW);
    digitalWrite(leftBackwardPin,HIGH);
  }

  if(mapped_right_vel > 255){
    analogWrite(rightMotorPwm,255);
  }
  else{
    analogWrite(rightMotorPwm,mapped_right_vel);
  }
  if(mapped_left_vel > 255){
    analogWrite(leftMotorPwm,255);
  }
  else{
    analogWrite(leftMotorPwm,mapped_left_vel);
  }
}

// fixes if wheels dont turn as expected velocity
void fix_velocities(){
  right_enc_pos = rightEncoder.encoderTicks;
  left_enc_pos = leftEncoder.encoderTicks;
  enc_velfix_time_now = millis();
  right_enc_vel = (absoluteValue(((right_enc_pos-right_enc_old_pos) / motor_one_turn_tick) * perimeter_of_wheels)/(enc_velfix_time_now-enc_velfix_time_old))*1000;// multiplied with 1000 for turning m/ms to m/s
  left_enc_vel  = (absoluteValue(((left_enc_pos -left_enc_old_pos)  / motor_one_turn_tick) * perimeter_of_wheels)/(enc_velfix_time_now-enc_velfix_time_old))*1000;// multiplied with 1000 for turning m/ms to m/s
  right_enc_old_pos = right_enc_pos;
  left_enc_old_pos = left_enc_pos;
  enc_velfix_time_old = enc_velfix_time_now;
  if (mapped_right_vel != 255){ //if it is 255 we can not increase anymore
    if (right_cmd_vel - right_enc_vel > 0.1){
      mapped_right_vel += (right_cmd_vel - right_enc_vel)*10;
      if (mapped_right_vel > 255){mapped_right_vel = 255;}
      analogWrite(rightMotorPwm,mapped_right_vel);
    }
  }
  if (right_enc_vel - right_cmd_vel > 0.1){
    mapped_right_vel -= (right_enc_vel - right_cmd_vel)*10;
      if (mapped_right_vel < 0){mapped_right_vel = 0;}
      analogWrite(rightMotorPwm,mapped_right_vel);
  }
  if (mapped_left_vel != 255){  //if it is 255 we can not increase anymore
    if (left_cmd_vel - left_enc_vel > 0.1){
      mapped_left_vel += (left_cmd_vel - left_enc_vel)*10;
      if (mapped_left_vel > 255){mapped_left_vel = 255;}
      analogWrite(leftMotorPwm,mapped_left_vel);
    }
  }
  if (left_enc_vel - left_cmd_vel > 0.1){
    mapped_left_vel -= (left_enc_vel - left_cmd_vel)*10;
      if (mapped_left_vel < 0){mapped_left_vel = 0;}
      analogWrite(leftMotorPwm,mapped_left_vel);
  }
}

//measures the range and turns the value(cm)
float getRangeUltrasound(int trigPin,int echoPin){
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds. Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // duration is the time (in microseconds) from the sending of the ping to the reception of its echo off of an object.
  ultrasonic_duration = pulseIn(echoPin, HIGH);
  // convert the time into a distance
  return ultrasonic_duration/58; //    duration/29/2, return cm
}


inline double degToRad(double deg) {
  return deg * M_PI / 180.0;
}
