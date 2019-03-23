#include "trin_base/trin_hardware.h"

TrinHardware::TrinHardware(ros::NodeHandle nhi)
{
    nh = nhi;
    arduino.setPort("/dev/ttyACM0");
    arduino.open();
    arduino.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(500);
    arduino.setTimeout(to);

    hardware_interface::JointStateHandle left_wheel_joint("left_wheel", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(left_wheel_joint);

    hardware_interface::JointStateHandle right_wheel_joint("right_wheel", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(right_wheel_joint);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle left_wheel_vel(jnt_state_interface.getHandle("left_wheel"), &cmd[0]);
    jnt_vel_interface.registerHandle(left_wheel_vel);

    hardware_interface::JointHandle right_wheel_vel(jnt_state_interface.getHandle("right_wheel"), &cmd[1]);
    jnt_vel_interface.registerHandle(right_wheel_vel);

    registerInterface(&jnt_vel_interface);
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
    l = ((int32_t)buf[3] << 24) | ((int32_t)buf[2] << 16) | ((int32_t)buf[1] << 8) | ((int32_t)buf[0]);
    r = ((int32_t)buf[7] << 24) | ((int32_t)buf[6] << 16) | ((int32_t)buf[5] << 8) | ((int32_t)buf[4]);
    lv = ((int16_t)buf[9] << 8) | ((int16_t)buf[8]);
    rv = ((int16_t)buf[11] << 8) | ((int16_t)buf[10]);
    pos[0] = (double)r * 0.006413900601;
    pos[1] = (double)l * 0.006413900601;
    vel[0] = (double)lv / 1000;
    vel[1] = (double)rv / 1000;
    // printf("%f %f %f %f\n",pos[0], pos[1], vel[0], vel[1]);
    // printf("%f %f\n",pos[0], pos[1]);
    ReadMutex.unlock();
}

void TrinHardware::writeToHardware()
{
    WriteMutex.lock();
    uint8_t temp[3];
    uint16_t left, right;
    left = (cmd[0] * 10) + 400;
    right = (cmd[1] * 10) + 400;
    temp[0] = (0x03f8 & left) >> 3;
    temp[1] = ((left & 0x7) << 5);
    temp[1] |= (0x03e0 & right) >> 5;
    temp[2] = ((right & 0x1f) << 3);
    // printf("%f, %f\n", cmd[0], cmd[1]);
    arduino.write(temp, 3);
    WriteMutex.unlock();
}

TrinHardware::~TrinHardware()
{
    arduino.close();
}
