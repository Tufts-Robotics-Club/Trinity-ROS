#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#define min_rotation 1.5

ros::Publisher chatter_pub;

void chatterCallback(const geometry_msgs::Twist::ConstPtr& raw_msg)
{
  geometry_msgs::Twist msg = *raw_msg;
  if (msg.linear.x == 0.0){
    if (msg.angular.z < 0.0){
      if (msg.angular.z > -min_rotation)
        msg.angular.z = -min_rotation;
    } else if (msg.angular.z > 0.0){
      if (msg.angular.z < min_rotation)
        msg.angular.z = min_rotation;
    }
  }
  chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter");
  ros::NodeHandle n;
  chatter_pub = n.advertise<geometry_msgs::Twist>("/trin_base_controller/cmd_vel", 0);
  ros::Subscriber sub = n.subscribe("/cmd_vel", 0, chatterCallback);
  ros::spin();
  return 0;
}
