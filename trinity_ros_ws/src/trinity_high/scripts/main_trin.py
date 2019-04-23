#!/usr/bin/env python

import roslaunch
import rospy
from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import Twist

explore_launched = False
flame_sensors = []
RATE = 30
kp = 0.9
vp = 0.7

def flame_callback(data):
	global flame_sensors
	flame_sensors = data.data.split(" ")
	flame_sensors = [float(i)/1024 for i in flame_sensors]

def start_callback(data):
	global explore_launched
	if(explore_launched == False) and (data.data == True):
		#navigateToRoom()
		launch.start()
		explore_launched = True

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ubuntu/Documents/robot_ws/src/trinity_high/launch/explore.launch"])
rospy.Subscriber("/start", Bool, start_callback)
rospy.Subscriber("/flame_sensors", String, flame_callback)
vel_pub = rospy.Publisher("/trin_base_controller/cmd_vel", Twist, queue_size=0)
fire_pub = rospy.Publisher("/servo_toggle", Empty, queue_size=0)

def navigateToRoom():
	launch.start()

	while max(flame_sensors) <100:
		pass

	launch.shutdown()
	navigateToFlame()
	

def navigateToFlame():
	rospy.rostime.wallsleep(3)
	global flame_sensors, vel_pub
	rate = rospy.Rate(RATE)
	forward, value = 0, 1
	# while(forward < 0.9 and (value > 0.1 or value < -0.1
	# while(abs(vel.angular.z) < 0.2 and abs(vel.linear.x) < 0.2)
	for i in range(500):
		value = [-5, -4, -3, -2, 1, 1, 2, 3, 4, 5]
		value = sum([i*j for i,j in zip(value, flame_sensors)])
		forward = ((flame_sensors[5] + flame_sensors[6])/2)*0.8
		vel = Twist()
		vel.angular.z = kp * value
		vel.linear.x = vp * (0.7 - forward)
		vel_pub.publish(vel)
		rate.sleep()
	vel.angular.z = 0
	vel_pub.publish(vel)
	extinguishFlame()

def extinguishFlame():
	temp = Empty()
	fire_pub.publish(temp)
	vel = Twist()

	rate = rospy.Rate(RATE)
	vel.angular.z = 2
	for i in range(15):
		vel_pub.publish(vel)
		rate.sleep()

	for i in range(3):
		vel.angular.z = -2
		for i in range(30):
			vel_pub.publish(vel)
			rate.sleep()
		vel.angular.z = 2
		for i in range(30):
			vel_pub.publish(vel)
			rate.sleep()

	vel.angular.z = -2
	for i in range(15):
		vel_pub.publish(vel)
		rate.sleep()

	fire_pub.publish(temp)

navigateToRoom()