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
#include <opencv2/opencv.hpp>

const std::string IMAGE_TOPIC= "/image/compressed";
const std::string LASER_TOPIC = "/scan";

const std::string ANGLE_TOPIC = "/Trinity/flame/angle";
const std::string FIREPOINT_TOPIC = "/Trinity/flame/point";
const std::string INTENSITY_TOPIC = "/Trinity/flame/intensity";

const float CAM_FOV = 62.2;

const int HUE_LOW = 0;
const int HUE_HI = 32;
const int SAT_LOW = 23;
const int SAT_HI = 128;
const int VAL_LOW = 230;
const int VAL_HI = 255;

const float INTENSITY_FACTOR = 1;

//Debugging Stuff
// const std::string DEBUG_PATH = "/home/faizan/flame_debug/";
// const std::string IN_PATH = "/home/faizan/flame_debug/1.jpg";


sensor_msgs::LaserScan latestScan;
cv_bridge::CvImagePtr latestImage;

ros::Publisher anglePub;
ros::Publisher pointPub;
ros::Publisher intnPub;

// void debugSave(cv::Mat in_img, std::string fname){
// 	cv::imwrite(DEBUG_PATH + fname + ".jpg", in_img);
// }

float find_angle(){
	std_msgs::Float32 angle;
	std_msgs::Float32 intensity;

	cv::cvtColor(latestImage->image, latestImage->image, cv::COLOR_BGR2HSV);
	cv::Mat filtered(latestImage->image.size(), CV_8U);
	inRange(latestImage->image, cv::Scalar(HUE_LOW, SAT_LOW, VAL_LOW), cv::Scalar(HUE_HI, SAT_HI, VAL_HI), filtered);

	cv::Moments m = cv::moments(filtered, true);
	float center_y = m.m10/m.m00;
	angle.data = (center_y - filtered.cols/2) * (CAM_FOV/filtered.cols);
	intensity.data = m.m00 / INTENSITY_FACTOR;

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

// void debugRun(){
// 	ROS_INFO("Running Debug\n");
// 	latestImage = boost::make_shared<cv_bridge::CvImage>();
// 	latestImage->image = cv::imread(IN_PATH, CV_LOAD_IMAGE_COLOR);

// 	float angle = find_angle();
// }

int main(int argc, char **argv){
	ros::init(argc, argv, "flame_detection");
	ros::NodeHandle n;
	
	ros::Subscriber scanSub = n.subscribe(LASER_TOPIC, 0, scanCallback);
	ros::Subscriber imageSub = n.subscribe(IMAGE_TOPIC, 0, imageCallback);

	anglePub = n.advertise<std_msgs::Float32>(ANGLE_TOPIC, 0);
	pointPub = n.advertise<geometry_msgs::Point>(FIREPOINT_TOPIC, 0);
	intnPub = n.advertise<std_msgs::Float32>(INTENSITY_TOPIC, 0);

	ros::spin();

	return 0;
}