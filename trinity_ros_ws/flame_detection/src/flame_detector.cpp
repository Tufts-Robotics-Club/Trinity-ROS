//Faizan Muhammad
//Trinity Firefighting 2019
//Tufts Robotics Club

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"

#include <cv_bridge/cv_bridge.h>

const std::string IMAGE_TOPIC= "/image/compressed";
const std::string LASER_TOPIC = "/scan";

const std::string ANGLE_TOPIC = "/Trinity/flame/angle";
const std::string FIREPOINT_TOPIC = "/Trinity/flame/point";
const std::string INTENSITY_TOPIC = "/Trinity/flame/intensity";

sensor_msgs::LaserScan latestScan;
cv_bridge::CvImagePtr latestImage;

ros::Publisher anglePub;
ros::Publisher pointPub;
ros::Publisher intnPub;

float find_angle(){
	std_msgs::Float32 angle;
	std_msgs::Float32 intensity;

	anglePub.publish(angle);
	intnPub.publish(intensity);

	return angle.data;
}

void find_point(float angle){
	geometry_msgs::Point i;
	pointPub.publish(i);
}

void find_flame(){
	float angle = find_angle();
	find_point(angle);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& inMsg){
	latestScan = *inMsg;
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& inMsg){
	latestImage = cv_bridge::toCvCopy(inMsg);
	find_flame();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "flame_detection");
	ros::NodeHandle n;
	
	ros::Subscriber scanSub = n.subscribe(LASER_TOPIC, 0, scanCallback);
	ros::Subscriber imageSub = n.subscribe(IMAGE_TOPIC, 0, imageCallback);

	anglePub = n.advertise<std_msgs::Float32>(ANGLE_TOPIC, 0);
	pointPub = n.advertise<geometry_msgs::Point>(FIREPOINT_TOPIC, 0);
	intnPub = n.advertise<std_msgs::Float32>(FIREPOINT_TOPIC, 0);

	ros::spin();

	return 0;
}