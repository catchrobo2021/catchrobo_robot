#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <catchrobo_control/catchrobo_hw.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trobo_control");
  ros::NodeHandle nh;

  MyRobot my_robot;
  controller_manager::ControllerManager cm(&my_robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::Duration dt(0.01);

    // my_robot.read(now, dt);
    // cm.update(now, dt);

    // my_robot.write(now, dt);
    // dt.sleep();

    my_robot.read(now, dt);
    cm.update(now, dt);

    my_robot.write(now, dt);
    dt.sleep();
  }
  spinner.stop();

  return 0;
}
