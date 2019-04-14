#include "trin_base/trin_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"
#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(TrinHardware &hardware,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    hardware.readFromHardware();
    cm.update(ros::Time::now(), elapsed);
    hardware.writeToHardware();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trinity_base");
    ros::NodeHandle nh;

    double control_frequency = 30.0;

    TrinHardware hardware(nh);
    controller_manager::ControllerManager cm(&hardware, nh);

    ros::Duration(1).sleep();
    ros::CallbackQueue trin_queue;
    ros::AsyncSpinner trin_spinner(1, &trin_queue);

    time_source::time_point last_time = time_source::now();
    ros::TimerOptions control_timer(
        ros::Duration(1 / control_frequency),
        boost::bind(controlLoop, boost::ref(hardware), boost::ref(cm), boost::ref(last_time)),
        &trin_queue);
    ros::Timer control_loop = nh.createTimer(control_timer);

    trin_spinner.start();

    ros::spin();

  return 0;
}