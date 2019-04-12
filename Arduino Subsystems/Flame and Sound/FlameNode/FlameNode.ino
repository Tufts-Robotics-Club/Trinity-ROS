#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

std_msgs::Int32MultiArray flame_sensor_msg;
ros:Publisher<std_msgs::Int32MultiArray> pub("flame_sensors", &flame_sensor_msg);

void extinguisher_motor_callback(const std_msgs::Bool& msg) {
  if (msg.data == true) {
        //digitalWrite(13, HIGH-digitalRead(13));
  }
  else {
        //do something
  }
}

//Read digital pins to see 
void updateFlameSensorMsg(){
  
}



//Check the flame message to see if any is on
void checkFlameDetected(){
  
}

ros::Subscriber<std_msgs::Bool> sub1("extinguish_flame", &extinguisher_motor_callback);

void setup() {
 nh.initNode();
 nh.advertise(pub);  
 nh.subscribe(sub1);
 nh.subscribe(sub2);
 
}
void loop() {
 updateFlameSensorMsg();
 checkFlameDetected();
 checkSoundStart();
 pub.publish(&flame_sensor_msg);
 nh.spinOnce(); 
 delay(50);
}
