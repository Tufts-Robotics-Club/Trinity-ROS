#ifndef TRIN_HARDWARE_H
#define TRIN_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <serial/serial.h>
#include <mutex>
#include "ros/ros.h"

class TrinHardware : public hardware_interface::RobotHW
{
public:
    TrinHardware(ros::NodeHandle nh);
    ~TrinHardware();
    void readFromHardware();
    void writeToHardware();
private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    ros::NodeHandle nh;
    serial::Serial arduino;
    std::mutex WriteMutex;
    std::mutex ReadMutex;
    double cmd[2] = {0,0};
    double pos[2] = {0,0};
    double vel[2] = {0,0};
    double eff[2] = {0,0};
};

#endif