#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <catchrobo_control/catchrobo_hw.h>
#include <iostream> // for debug
#include <math.h>

MyRobo::MyRobo()
{
  for (size_t i = 0; i < joint_num; i++)
  {
    std::string joint_name = "arm/joint" << i;
    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_interface.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface);

  for (size_t i = 0; i < joint_num; i++)
  {
    std::string joint_name = "arm/joint" << i;
    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_name), &cmd_[i]);
    jnt_cmd_interface.registerHandle(pos_handle);
  }
  registerInterface(&jnt_cmd_interface);
}

void MyRobo::read()
{
  ////read from odrive
}

void MyRobo::write()
{

  ////write for odrive
  for (size_t i = 0; i < joint_num; i++)
  {
    pos_[i] = cmd_[i];
  }
}
