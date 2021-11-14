#!/usr/bin/env python3
import rospy
import math
import can
import cantools
import time



db = cantools.database.load_file("odrive-cansimple.dbc")
# print(db)

# bus = can.Bus("vcan0", bustype="virtual")
bus = can.Bus("can0", bustype="socketcan")
axisID = 0x01


        
rospy.init_node("catcrobo_driver")


r = rospy.Rate(5)
while not rospy.is_shutdown():

    motor_state = [0]*3 # position, velocity, effort
    for msg in bus:
        if msg.arbitration_id == ((axisID << 5) | db.get_message_by_name('Get_Encoder_Estimates').frame_id):
            motor_state[0] = db.decode_message('Get_Encoder_Estimates', msg.data)['Pos_Estimate']
            motor_state[1] = db.decode_message('Get_Encoder_Estimates', msg.data)['Vel_Estimate']
            break
        if msg.arbitration_id == ((axisID << 5) | db.get_message_by_name('Get_Iq').frame_id):
            motor_state[2] = db.decode_message('Get_Iq', msg.data)['Iq_Measured']
            break
    print(motor_state)

    #r.sleep()