#include <ros.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 distance_msg;
ros::Publisher pub_distance("distance", &distance_msg);
ros::NodeHandle nh;

int trigPin=13; //Sensor Trip pin connected to Arduino pin 13
int echoPin=11;  //Sensor Echo pin connected to Arduino pin 11
float pingTime;  //time for ping to travel from sensor to target and return
float distance; //Distance to Target in meters
float speedOfSound=1236; //Speed of sound in miles per hour when temp is 77 degrees. 776.5miles
 
 
void setup() {
  
  nh.initNode();
  nh.advertise(pub_distance);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
 
}

long publisher_timer;

 
void loop() {
  
  digitalWrite(trigPin, LOW); //Set trigger pin low
  delayMicroseconds(2000); //Let signal settle
  digitalWrite(trigPin, HIGH); //Set trigPin high
  delayMicroseconds(15); //Delay in high state
  digitalWrite(trigPin, LOW); //ping has now been sent
  delayMicroseconds(10); //Delay in low state
  
  pingTime = pulseIn(echoPin, HIGH);  //pingTime is presented in microceconds
  pingTime=pingTime/1000000; //convert pingTime to seconds by dividing by 1000000 (microseconds in a second)
  pingTime=pingTime/3600; //convert pingtime to hourse by dividing by 3600 (seconds in an hour)
  distance= speedOfSound * pingTime;  //This will be in miles, since speed of sound was miles per hour
  distance= distance/2; //Remember ping travels to target and back from target, so you must divide by 2 for actual target distance.
  distance= distance*1000;    //Convert miles to inches by multipling by 1609 (meter per mile)
  
  distance_msg.data = distance;
  pub_distance.publish(&distance_msg);

  publisher_timer = millis() + 4000; //publish once a second

  nh.spinOnce();
  
}


