#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32,Bool
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
        
        self._time_start = 0
        self._game_has_started = False

        self._joint1_position_publisher = rospy.Publisher('joint1/position', Float32, queue_size=10)
        self._joint1_effort_publisher = rospy.Publisher('joint1/effort', Float32, queue_size=10)
        self._joint2_position_publisher = rospy.Publisher('joint2/position', Float32, queue_size=10)
        self._joint2_effort_publisher = rospy.Publisher('joint2/effort', Float32, queue_size=10)
        self._joint3_position_publisher = rospy.Publisher('joint3/position', Float32, queue_size=10)
        self._joint3_effort_publisher = rospy.Publisher('joint3/effort', Float32, queue_size=10)
        self._joint4_position_publisher = rospy.Publisher('joint4/position', Float32, queue_size=10)
        self._joint4_effort_publisher = rospy.Publisher('joint4/effort', Float32, queue_size=10)
        self._time_left_publisher = rospy.Publisher('time_left', Float32, queue_size=10)

        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
        #rospy.Subscriber("game_start", Bool, self.game_start_callback)
        #rospy.Timer(rospy.Duration(0.1), self.timer_callback)
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


    def game_start_callback(self,data):
        if data.data is True:
            self._time_start = rospy.Time.now()
            self._game_has_started = True
        else: 
            pass


    def timer_callback(self,event):
        if self._game_has_started is True:
            time_left = 180 - (rospy.Time.now()-self._time_start)
        else:
            time_left = 180
        self._time_left_publisher.publish(time_left)
    
 
if __name__ == "__main__":
    state_publisher()