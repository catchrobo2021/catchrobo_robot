#!/usr/bin/env python3
#from odrive_node import ODriveNode
from sensor_msgs.msg import JointState
import time
import rospy
#import odrive
#from odrive.enums import *
#from odrive.utils import *
import can
import cantools




class ODriveBridgeCAN:
    def __init__(self, MOTOR_NUM, config):

        self.MOTOR_NUM = MOTOR_NUM # number of joints
        self._dbc = cantools.database.load_file("/home/yutakage/catkin_ws/src/catchrobo_robot/catchrobo_driver/scripts/odrive-cansimple.dbc")
        self._bus = can.Bus("can0", bustype="socketcan")
        self._axis_id = [0x01,0x02,0x3,0x4]


    def search_index(self,joint):
        #print("\nRequesting Index Search (0x03) on axisID: " + str(axisID))
        msg = self._dbc.get_message_by_name('Set_Axis_State')
        data = msg.encode({'Axis_Requested_State': 0x06})
        msg = can.Message(arbitration_id=msg.frame_id | self._axis_id[joint] << 5, is_extended_id=False, data=data)
        #print(self._dbc.decode_message('Set_Axis_State', msg.data))
        #print(msg)

        try:
            self._bus.send(msg)
            print("Message sent on {}".format(self._bus.channel_info))
        except can.CanError:
            print("Message NOT sent!  Please verify can0 is working first")

        print("Waiting for calibration to finish...")
        # Read messages infinitely and wait for the right ID to show up
        while True:
            msg = self._bus.recv()
            if msg.arbitration_id == ((self._axis_id[joint] << 5) | self._dbc.get_message_by_name('Heartbeat').frame_id):
                current_state = self._dbc.decode_message('Heartbeat', msg.data)['Axis_State']
                if current_state == 0x1:
                    print("\nAxis has returned to Idle state.")
                    break

        for msg in self._bus:
            if msg.arbitration_id == ((self._axis_id[joint] << 5) | self._dbc.get_message_by_name('Heartbeat').frame_id):
                errorCode = self._dbc.decode_message('Heartbeat', msg.data)['Axis_Error']
                if errorCode == 0x00:
                    print("No errors")
                else:
                    print("Axis error!  Error code: "+str(hex(errorCode)))
                break
        return True

        
    def engage(self, joint, index_search=False):
        if joint < self.MOTOR_NUM :
            data = self._dbc.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
            msg = can.Message(arbitration_id=0x07 | self._axis_id[joint] << 5, is_extended_id=False, data=data)
            try:
                self._bus.send(msg)
                #print("Message sent on {}".format(self._bus.channel_info))
            except can.CanError:
                pass
                print("Message NOT sent!")

            for msg in self._bus:
                if msg.arbitration_id == 0x01 | self._axis_id[joint] << 5:
                    #print("\nReceived Axis heartbeat message:")
                    msg = self._dbc.decode_message('Heartbeat', msg.data)
                    #print(msg)
                    if msg['Axis_State'] == 0x8:
                        print("Axis has entered closed loop")
                    else:
                        print("Axis failed to enter closed loop")
                    break
        else:
            pass
            

    def idle(self, joint):
        if joint < self.MOTOR_NUM :
            data = self._dbc.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x01})
            msg = can.Message(arbitration_id=0x07 | self._axis_id[joint] << 5, is_extended_id=False, data=data)
            try:
                self._bus.send(msg)
                #print("Message sent on {}".format(self._bus.channel_info))
            except can.CanError:
                pass
                #print("Message NOT sent!")

            for msg in self._bus:
                if msg.arbitration_id == 0x01 | self._axis_id[joint] << 5:
                    #print("\nReceived Axis heartbeat message:")
                    msg = self._dbc.decode_message('Heartbeat', msg.data)
                    #print(msg)
                    if msg['Axis_State'] == 0x1:
                        print("Axis has entered closed loop")
                    else:
                       print("Axis failed to enter closed loop")
                    break
        else:
            pass



    def write(self,motor_control):
        for i in range(self.MOTOR_NUM):
            data = self._dbc.encode_message('Set_Input_Pos', {'Input_Pos':motor_control.position[i], 'Vel_FF':motor_control.velocity[i], 'Torque_FF':motor_control.effort[i]})
            msg = can.Message(arbitration_id=self._axis_id[i] << 5 | 0x00C, data=data, is_extended_id=False)
            self._bus.send(msg)


    def read(self):
        motor_state = JointState()
        motor_state.name = [""] * self.MOTOR_NUM
        motor_state.position = [0] * self.MOTOR_NUM
        motor_state.velocity = [0] * self.MOTOR_NUM
        motor_state.effort = [0] * self.MOTOR_NUM

        for i in range(self.MOTOR_NUM):
            for msg in self._bus:
                if msg.arbitration_id == ((self._axis_id[i] << 5) | self._dbc.get_message_by_name('Get_Encoder_Estimates').frame_id):
                    motor_state.position[i] = self._dbc.decode_message('Get_Encoder_Estimates', msg.data)['Pos_Estimate']
                    motor_state.velocity[i] = self._dbc.decode_message('Get_Encoder_Estimates', msg.data)['Vel_Estimate']
                    break
        
            #msg = can.Message(arbitration_id=0x014 | self._axis_id[i] << 5, is_extended_id=False,is_remote_frame=True)
            #try:
            #    self._bus.send(msg)
            #except can.CanError:
            #    pass
            #    print("Message NOT sent!")
            #for msg in self._bus:
            #    if msg.arbitration_id == ((self._axis_id[i] << 5) | self._dbc.get_message_by_name('Get_Iq').frame_id):
            #        motor_state.effort[i] = self._dbc.decode_message('Get_Iq', msg.data)['Iq_Measured']
            #        break
        return motor_state


    def communicate(self,motor_control):
        motor_state = JointState()
        motor_state.name = [""] * self.MOTOR_NUM
        motor_state.position = [0] * self.MOTOR_NUM
        motor_state.velocity = [0] * self.MOTOR_NUM
        motor_state.effort = [0] * self.MOTOR_NUM
        
        for i in range(self.MOTOR_NUM):
            data = self._dbc.encode_message('Set_Input_Pos', {'Input_Pos':motor_control.position[i], 'Vel_FF':motor_control.velocity[i], 'Torque_FF':motor_control.effort[i]})
            msg = can.Message(arbitration_id=self._axis_id[i] << 5 | 0x00C, data=data, is_extended_id=False)
            self._bus.send(msg)
            for msg in self._bus:
                if msg.arbitration_id == ((self._axis_id[i] << 5) | self._dbc.get_message_by_name('Get_Encoder_Estimates').frame_id):
                    motor_state.position[i] = self._dbc.decode_message('Get_Encoder_Estimates', msg.data)['Pos_Estimate']
                    motor_state.velocity[i] = self._dbc.decode_message('Get_Encoder_Estimates', msg.data)['Vel_Estimate']
                    break
                if msg.arbitration_id == ((self._axis_id[i] << 5) | self._dbc.get_message_by_name('Get_Iq').frame_id):
                    motor_state.effort[i] = self._dbc.decode_message('Get_Iq', msg.data)['Iq_Measured']
                    break
        
        return motor_state

        

    def read_vbus_voltage(self):
        msg = can.Message(arbitration_id=0x017 | self._axis_id[0] << 5, is_extended_id=False,is_remote_frame=True)
        try:
            self._bus.send(msg)
        except can.CanError:
            pass
            print("Message NOT sent!")
        voltage = 0
        for msg in self._bus:
            if msg.arbitration_id == ((self._axis_id[0] << 5) | self._dbc.get_message_by_name('Get_Vbus_Voltage').frame_id):
                voltage = self._dbc.decode_message('Get_Vbus_Voltage', msg.data)['Vbus_Voltage']
                break
        return voltage


    def engage_all(self,index_search=False):
        for i in range(self.MOTOR_NUM):
            self.engage(joint=i, index_search=index_search)


    def idle_all(self):
        for i in range(self.MOTOR_NUM):
            self.idle(joint=i)


    def search_index_all(self):
        for i in range(self.MOTOR_NUM):
            self.search_index(joint=i)


    def hard_stop(self,joint,tolerance=0.001,direction=1,velocity=0.1):
        self.engage(joint=joint,index_search=False)
        while(True):
            msg = can.Message(arbitration_id=0x014 | self._axis_id[joint] << 5, is_extended_id=False,is_remote_frame=True)
            try:
                self._bus.send(msg)
            except can.CanError:
                pass
                print("Message NOT sent!")
            for msg in self._bus:

                print(msg.arbitration_id,((self._axis_id[joint] << 5) | self._dbc.get_message_by_name('Get_Iq').frame_id))
                if msg.arbitration_id == ((self._axis_id[joint] << 5) | self._dbc.get_message_by_name('Get_Iq').frame_id):
                    effort = self._dbc.decode_message('Get_Iq', msg.data)['Iq_Measured']
                    print("found")
                    break

        motor_state = self.read()
        target_position = motor_state.position[joint]
        position_old = motor_state.position[joint]
        counter = 0
        
        while True:
            # move
            target_position += direction * velocity * 0.01
            data = self._dbc.encode_message('Set_Input_Pos', {'Input_Pos':target_position, 'Vel_FF':0.0, 'Torque_FF':0.0})
            msg = can.Message(arbitration_id=self._axis_id[joint] << 5 | 0x00C, data=data, is_extended_id=False)
            self._bus.send(msg)
            #time.sleep(0.01)

            # get position
            motor_state = self.read()
            if abs(motor_state.position[joint] - position_old) < tolerance:
                counter += 1

                print("detected")
                print(abs(motor_state.position[joint] - position_old))
            else:
                counter = 0
            
            if counter > 3:
                self.idle(joint=joint)
                break
            position_old = motor_state.position[joint]
        self.search_index(joint=joint)