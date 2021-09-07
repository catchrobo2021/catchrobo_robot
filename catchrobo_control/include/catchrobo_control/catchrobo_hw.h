#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot();
  void read(ros::Time, ros::Duration);

  void write(ros::Time, ros::Duration);

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_cmd_interface;
  static const int joint_num = 5;
  double cmd_[5];
  double pos_[5];
  double vel_[5];
  double eff_[5];
};