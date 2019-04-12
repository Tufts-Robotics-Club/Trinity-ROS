#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

std_msgs::Int32MultiArray flame_sensor_msg;
ros:Publisher<std_msgs::Int32MultiArray> pub("flame_sensor_led", &flame_sensor_msg);

void extinguisher_motor_callback(const std_msgs::Bool& msg) {
  if (msg.data == true) {
        //digitalWrite(13, HIGH-digitalRead(13));
  }
  else {
    //do something
  }
}

void game_state_callback(const std_msgs::Int32MultiArray msg) {
 if (msg.data[0] == 1) {
   //do something
 }
 if (msg.data[1] == 1) {
   //do something 
 }
 //blah blah blah
}

ros::Subscriber<std_msgs::Bool> sub1("extinguisher_motor_on", &extinguisher_motor_callback);
ros::Subscriber<std_msgs::Int32MultiArray> sub2("game_state", &game_state_callback);

void setup() {
  //pinmode?
  nh.initNode();
 nh.advertise(pub);  
 nh.subscribe(sub1);
 nh.subscribe(sub2);
 
}
void loop() {
 pub.publish(&flame_sensor_msg);
 nh.spinOnce(); 
 //delay(1);
}
