#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>

class MyRobo : public hardware_interface::RobotHW
{
public:
  TRobo();

  // ros::Time getTime() const { return ros::Time::now(); }
  // ros::Duration getPeriod() const { return ros::Duration(0.01); }

  // void read(ros::Time, ros::Duration);
  void read();
  void write();

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_cmd_interface;
  const int joint_num = 5;
  double cmd_[joint_num];
  double pos_[joint_num];
  double vel_[joint_num];
  double eff_[joint_num];
};
