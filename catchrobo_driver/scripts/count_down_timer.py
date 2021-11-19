#!/usr/bin/env python
import rospy
import datetime
from std_msgs.msg import Float32,Bool,String

class count_down_timer:
    def __init__(self):
        rospy.init_node('count_down_timer')

        # self._time_left_publisher = rospy.Publisher('time_left', Float32, queue_size=10)
        self._time_left_publisher = rospy.Publisher('time_left', String, queue_size=10)  
        self._time_start = 0
        self._game_has_started = False

        rospy.Subscriber("pause", Bool, self.game_start_callback)
        rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.spin()

    def game_start_callback(self,data):
        if self._game_has_started is False and data.data is False:
            self._time_start = rospy.get_time()
            self._game_has_started = True
        else: 
            pass

    def timer_callback(self,event):
        if self._game_has_started is False:
            time_left = 180
        else:
            time_left = 180 - (rospy.get_time() - self._time_start)
            if time_left < 0:
                time_left = 0
        self._time_left_publisher.publish(str(int(time_left/60)).zfill(2)  + ":" + str(int(time_left%60)).zfill(2) + ":" + str(int((time_left - int(time_left))*100)).zfill(2))
        #self._time_left_publisher.publish(str(int(time_left/60)).zfill(2)  + ":" + str(int(time_left%60)).zfill(2) )
        

if __name__ == "__main__":
    count_down_timer()