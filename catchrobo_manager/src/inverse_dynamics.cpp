#include <moveit/dynamics_solver/dynamics_solver.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    //   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // std::stringstream ss;
        // ss << "hello world " << count;
        DynamicsSolver
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}