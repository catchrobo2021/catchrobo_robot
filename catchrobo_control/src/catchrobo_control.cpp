#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <catchrobo_control/catchrobo_hw.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trobo_control");
  ros::NodeHandle nh;

  MyRobo my_robot;
  controller_manager::ControllerManager cm(&my_robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    ros::Time now = my_robot.getTime();
    ros::Duration dt = my_robot.getPeriod();

    // my_robot.read(now, dt);
    // cm.update(now, dt);

    // my_robot.write(now, dt);
    // dt.sleep();

    my_robot.read();
    cm.update(now, dt);

    my_robot.write();
    dt.sleep();
  }
  spinner.stop();

  return 0;
}
