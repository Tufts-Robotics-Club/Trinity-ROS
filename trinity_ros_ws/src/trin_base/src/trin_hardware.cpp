#include "trin_base/trin_hardware.h"

#define PORT "/dev/ttyACM0"
#define BAUD_RATE 115200
#define TIMEOUT 500

TrinHardware::TrinHardware(ros::NodeHandle nhi)
{
    nh = nhi;
    arduino.setPort(PORT);
    
    try {
        arduino.open();
    } catch(serial::IOException &err) {
        ROS_ERROR_STREAM("Connection failed with port: " << arduino.getPort());
        ros::shutdown();
    }

    ROS_INFO("Connection established with Robot");
    arduino.setBaudrate(BAUD_RATE);
    serial::Timeout to = serial::Timeout::simpleTimeout(TIMEOUT);
    arduino.setTimeout(to);

    hardware_interface::JointStateHandle left_joint("left_wheel", &pos[0], &vel[0], &eff[0]);
    state_interface.registerHandle(left_joint);

    hardware_interface::JointStateHandle right_joint("right_wheel", &pos[1], &vel[1], &eff[1]);
    state_interface.registerHandle(right_joint);

    registerInterface(&state_interface);

    hardware_interface::JointHandle left_vel(state_interface.getHandle("left_wheel"), &cmd[0]);
    vel_interface.registerHandle(left_vel);

    hardware_interface::JointHandle right_vel(state_interface.getHandle("right_wheel"), &cmd[1]);
    vel_interface.registerHandle(right_vel);

    registerInterface(&vel_interface);
}

void TrinHardware::readFromHardware()
{
    ReadMutex.lock();
    int32_t l, r;
    int16_t lv, rv;
    uint8_t temp[3] = {0xff, 0, 0};
    uint8_t buf[12];
    arduino.write(temp, 3);
    arduino.read(buf, 12);
    l = (int32_t)buf[3]<<24 | (int32_t)buf[2]<<16 | (int32_t)buf[1]<<8 | (int32_t)buf[0];
    r = (int32_t)buf[7]<<24 | (int32_t)buf[6]<<16 | (int32_t)buf[5]<<8 | (int32_t)buf[4];
    lv = (int16_t)buf[9] << 8 | (int16_t)buf[8];
    rv = (int16_t)buf[11] << 8 | (int16_t)buf[10];
    pos[0] = (double)l * 0.006413900601;
    pos[1] = (double)r * 0.006413900601;
    vel[0] = (double)lv / 1000;
    vel[1] = (double)rv / 1000;
    ReadMutex.unlock();
}

void TrinHardware::writeToHardware()
{
    WriteMutex.lock();
    uint8_t temp[3];
    uint16_t left, right;
    left = (cmd[0] * 10) + 1024;
    right = (cmd[1] * 10) + 1024;
    temp[0] = (0x07f0 & left) >> 4;
    temp[1] = ((left & 0xf) << 4);
    temp[1] |= (0x0780 & right) >> 7;
    temp[2] = ((right & 0x7f) << 1);
    arduino.write(temp, 3);
    WriteMutex.unlock();
}

TrinHardware::~TrinHardware()
{
    arduino.close();
}
