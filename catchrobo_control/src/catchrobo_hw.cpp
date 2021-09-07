#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <catchrobo_control/catchrobo_hw.h>
#include <iostream> // for debug
#include <math.h>

#include <sstream>

MyRobot::MyRobot()
{
  for (size_t i = 0; i < joint_num; i++)
  {
    std::stringstream ss;
    ss << "arm/joint" << i;
    std::string joint_name = ss.str();
    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_interface.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface);

  for (size_t i = 0; i < joint_num; i++)
  {
    std::stringstream ss;
    ss << "arm/joint" << i;
    std::string joint_name = ss.str();
    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_name), &cmd_[i]);
    jnt_cmd_interface.registerHandle(pos_handle);
  }
  registerInterface(&jnt_cmd_interface);
}

void MyRobot::read(ros::Time time, ros::Duration period)
{
  ////read from odrive
}

void MyRobot::write(ros::Time time, ros::Duration period)
{

  ////write for odrive
  for (size_t i = 0; i < joint_num; i++)
  {
    pos_[i] = cmd_[i];
  }
}
