#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class state_publisher:
    def __init__(self):
        rospy.init_node('state_publisher')
        self.MOTOR_NUM = 4 # number of joints
        # joint state
        joint_state = JointState()
        joint_state.name = [""] * self.MOTOR_NUM
        joint_state.position = [0] * self.MOTOR_NUM
        joint_state.velocity = [0] * self.MOTOR_NUM
        joint_state.effort = [0] * self.MOTOR_NUM
        self._joint_state = joint_state

        self._joint1_position_publisher = rospy.Publisher('joint1/position', Float32, queue_size=10)
        self._joint1_effort_publisher = rospy.Publisher('joint1/effort', Float32, queue_size=10)
        self._joint2_position_publisher = rospy.Publisher('joint2/position', Float32, queue_size=10)
        self._joint2_effort_publisher = rospy.Publisher('joint2/effort', Float32, queue_size=10)
        self._joint3_position_publisher = rospy.Publisher('joint3/position', Float32, queue_size=10)
        self._joint3_effort_publisher = rospy.Publisher('joint3/effort', Float32, queue_size=10)
        self._joint4_position_publisher = rospy.Publisher('joint4/position', Float32, queue_size=10)
        self._joint4_effort_publisher = rospy.Publisher('joint4/effort', Float32, queue_size=10)

        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
        
        rospy.spin()


    def joint_state_callback(self,data):
        self._joint1_position_publisher.publish(data.position[0])
        self._joint1_effort_publisher.publish(abs(data.effort[0]))
        self._joint2_position_publisher.publish(data.position[1])
        self._joint2_effort_publisher.publish(abs(data.effort[1]))
        self._joint3_position_publisher.publish(data.position[2])
        self._joint3_effort_publisher.publish(abs(data.effort[2]))
        self._joint4_position_publisher.publish(data.position[3])
        self._joint4_effort_publisher.publish(abs(data.effort[3]))
 
if __name__ == "__main__":
    state_publisher()