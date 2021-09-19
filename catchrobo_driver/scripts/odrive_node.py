#!/usr/bin/env python3
from __future__ import print_function
from os import wait

import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *
from odrive.utils import * 

import fibre

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt 
import threading

#import roslib

import std_msgs.msg
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import std_srvs.srv

#roslib.load_manifest('diagnostic_updater')
import diagnostic_updater, diagnostic_msgs.msg

import math

#from odrive_interface import ODriveInterfaceAPI, ODriveFailure
#from odrive_interface import ChannelBrokenException, ChannelDamagedException

logging.basicConfig()
default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.INFO)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)


class ODriveNode(object):

    def __init__(self):
        self.logger = default_logger
        self.cpr = 2**14
        self.driver = None

    def connect_test(self, timeout=30):
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        try:
            #self.driver = odrive.find_any(timeout=timeout, logger=self.logger)
            self.driver = odrive.find_all()
            self.axes = (self.driver.axis0, self.driver.axis1)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
                        
        # save some parameters for easy access
        self.right_axis = self.driver.axis0
        self.left_axis  = self.driver.axis1
        
        # check for no errors
        for axis in [self.right_axis, self.left_axis]:
            if axis.error != 0:
                error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                self.logger.error(error_str)
                self.reboot()
                return False
        self.logger.info("Connected to ODrive. " + self.get_version_string())
        return True
    
    def get_version_string(self):
        if not self.driver:
            return "Not connected."
        return "ODrive %s, hw v%d.%d-%d, fw v%d.%d.%d%s, sdk v%s" % (
            str(self.driver.serial_number),
            self.driver.hw_version_major, self.driver.hw_version_minor, self.driver.hw_version_variant,
            self.driver.fw_version_major, self.driver.fw_version_minor, self.driver.fw_version_revision,
            "-dev" if self.driver.fw_version_unreleased else "",
            odrive.version.get_version_str())
    
    def reboot(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        try:
            self.driver.reboot()
        except KeyError:
            self.logger.error("Rebooted ODrive.")
        except:
            self.logger.error("Failed to reboot")
        finally:
            self.driver = None
        return True

    def connect(self, serial_number, timeout = 10):
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        try:
            self.driver = odrive.find_any(serial_number=serial_number, timeout=timeout, logger=self.logger)
            self.axes = (self.driver.axis0, self.driver.axis1)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
        self.right_axis = self.driver.axis0
        self.left_axis  = self.driver.axis1
        
        # check for no errors
        for axis in [self.right_axis, self.left_axis]:
            if axis.error != 0:
                error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                self.logger.error(error_str)
                self.reboot()
                return False
        self.logger.info("Connected to ODrive. " + self.get_version_string())
        return True

    def disconnect(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        self.logger.info("Disconnect")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE
        self.driver = None
        return True

    def get_errors(self, clear=True, mode=0):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        if not self.driver:
            self.logger.error("Not connected.")
            return None
            
        axis_error = self.axes[0].error or self.axes[1].error
        
        if axis_error:
            error_string = "Errors(hex): L: a%x m%x e%x c%x, R: a%x m%x e%x c%x" % (
                self.left_axis.error,  self.left_axis.motor.error,  self.left_axis.encoder.error,  self.left_axis.controller.error,
                self.right_axis.error, self.right_axis.motor.error, self.right_axis.encoder.error, self.right_axis.controller.error,
            )
            self.logger.error(error_string)
            self.logger.error(dump_errors(self.driver))
        else:
            self.logger.info("Not error")
        
        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0
        
        if mode==1:
            dump_errors(self.driver)

        if axis_error:
            return error_string
    
    def set_config(self, axis=0, encoder_pin=4):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        try:
            self.axes[axis].motor.config.current_lim = 11
            self.axes[axis].motor.config.calibration_current = 5
            self.axes[axis].controller.config.vel_limit = 300
            self.driver.config.brake_resistance = 0.6
            self.axes[axis].motor.config.pole_pairs = 21
            self.axes[axis].motor.config.torque_constant = 0.07
            self.axes[axis].motor.config.resistance_calib_max_voltage = 6.0
            self.axes[axis].encoder.config.abs_spi_cs_gpio_pin = encoder_pin 
            self.axes[axis].encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
            self.axes[axis].encoder.config.cpr = 2**14
            self.axes[axis].controller.config.inertia  = 0
            self.axes[axis].motor.config.pre_calibrated = True
            self.axes[axis].encoder.config.pre_calibrated = True
            
            self.axes[axis].controller.config.input_filter_bandwidth = 2.0
            self.axes[axis].trap_traj.config.vel_limit = 1.0
            self.axes[axis].trap_traj.config.accel_limit = 1.0
            self.axes[axis].trap_traj.config.decel_limit = 1.0
            self.axes[axis].controller.config.inertia = 0

            self.driver.save_configuration()
        except:
            error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
            self.logger.error(error_str)
            return False
        self.driver.reboot()
        return True

    def reset_config(self): self.driver.erase_configuration()

    def calibrate(self, axis=0, mode=0):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        self.logger.info("Calibrating axis %d..." % axis)

        if mode:
            self.axes[axis].requested_state = AXIS_STATE_MOTOR_CALIBRATION
            time.sleep(1)
            while self.axes[axis].current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if self.axes[axis].error != 0:
                self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (self.axes[axis].error, self.axes[axis].motor.error))
                return False
            self.axes[axis].requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(1)
            while self.axes[axis].current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if self.axes[axis].error != 0:
                self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (self.axes[axis].error, self.axes[axis].motor.error))
                return False

        self.axes[axis].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(1)
        while self.axes[axis].current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        if self.axes[axis].error != 0:
            self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (self.axes[axis].error, self.axes[axis].motor.error))
            return False
        self.driver.save_configuration()
        return True

    def set_PID(self, axis=0, pos_P=20, vel_P=0.16, vel_I=0.33):
        self.axes[axis].controller.config.pos_gain = pos_P
        self.axes[axis].controller.config.vel_gain = vel_P
        self.axes[axis].controller.config.vel_integrator_gain = vel_I
        self.driver.save_configuration()
        return True
    
    def engage(self , axis=0 , mode=0):
        if mode == 1:
            self.search(axis)
            while self.axes[axis].current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
        self.axes[axis].requested_state  = AXIS_STATE_CLOSED_LOOP_CONTROL
        return True

    def idle(self, axis=0): self.axes[axis].requested_state = AXIS_STATE_IDLE

    def search(self, axis=0): self.axes[axis].requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

    def input_mode(self, axis=0, mode=1):
        #input_mode = 1 def
        #input_mode = 3 filter
        #input_mode = 5 trap
        self.axes[axis].controller.config.input_mode = mode

    def control_mode(self, axis=0, mode="POS"):
        if mode=="POS":
            self.axes[axis].controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        if mode=="VEL":
            self.axes[axis].controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        if mode=="TOR":
            self.axes[axis].controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

    def drive_pos(self, axis=0, val=0): self.axes[axis].controller.input_pos = val
    def drive_vel(self, axis=0, val=0): self.axes[axis].controller.input_vel = val
    def drive_tor(self, axis=0, val=0): self.axes[axis].controller.input_torque = val

    def get_current(self, axis=0):  return self.axes[axis].motor.current_control.Iq_measured
    def get_count(self, axis=0):  return self.axes[axis].encoder.shadow_count
    def get_pos(self, axis=0):  return self.axes[axis].encoder.pos_estimate
    def get_vel(self, axis=0):  return self.axes[axis].encoder.vel_estimate
    
    def set_start_closed(self, axis=0):
        self.axes[axis].config.startup_closed_loop_control = True
        self.driver.save_configuration()

    def config_endstop(self, axis=0, max_pin=6, min_pin=7):
        self.axes[axis].max_endstop.config.gpio_num = max_pin
        self.axes[axis].min_endstop.config.gpio_num = min_pin
        self.axes[axis].max_endstop.config.enabled = True
        self.axes[axis].min_endstop.config.enabled = True
        self.axes[axis].min_endstop.config.offset = 0.0
        self.axes[axis].max_endstop.config.debounce_ms = 50
        self.axes[axis].min_endstop.config.debounce_ms = 50
        self.axes[axis].max_endstop.config.is_active_high = False
        self.axes[axis].min_endstop.config.is_active_high = False
        self.axes[axis].min_endstop.config.pullup
        self.axes[axis].max_endstop.config.pullup
        self.driver.save_configuration()
        self.driver.reboot()
    
    def cancel_endstop(self, axis=0):
        self.axes[axis].max_endstop.config.enabled = False
        self.axes[axis].min_endstop.config.enabled = False

    def switch_test(self, axis=0):  return self.axes[axis].max_endstop.endstop_state , self.axes[axis].min_endstop.endstop_state

    def config_homing(self, axis=0):
        self.axes[axis].controller.config.homing_speed = 0.25
        self.axes[axis].controller.config.vel_ramp_rate = 0.1
        self.axes[axis].trap_traj.config.vel_limit = 0.1
        self.axes[axis].trap_traj.config.accel_limit = 0.1
        self.axes[axis].trap_traj.config.decel_limit = 0.1
        self.driver.save_configuration()

    def homing(self, axis=0):   self.axes[axis].requested_state = AXIS_STATE_HOMING

class Odrive_gui(object):

    def __init__(self, odrive_class, serial_number):
        self.odrive = odrive_class
        self.serial_number = serial_number

        self.stop_time = 0
        self.jobID = None

        self.root = tk.Tk()
        self.root.title(u"Odrive GUI")
        #self.root.geometry("1980x1080")

        frame_left = tk.Frame(self.root)
        frame_right = tk.Frame(self.root)
        
        frame_0 = tk.Frame(frame_left,  relief=tk.RIDGE, bd=2)
        frame_1 = tk.Frame(frame_left)
        frame_2 = tk.Frame(frame_left)
        frame_encoder = tk.Frame(frame_1,  relief=tk.RIDGE, bd=2)
        frame_axis = tk.Frame(frame_1,  relief=tk.RIDGE, bd=2)
        frame_mode = tk.Frame(frame_2,  relief=tk.RIDGE, bd=2)
        frame_motor = tk.Frame(frame_2,  relief=tk.RIDGE, bd=2)
        frame_val = tk.Frame(frame_left,  relief=tk.RIDGE, bd=2)
        frame_btn = tk.Frame(frame_left,  relief=tk.RIDGE, bd=2)
        frame_error = tk.Frame(frame_left,  relief=tk.RIDGE, bd=2)

        self.encoder_mode = tk.IntVar()
        self.encoder_mode.set(0)
        en_mode1 = tk.Radiobutton(frame_encoder, value=0, variable=self.encoder_mode, text='SPI', font=30)
        en_mode2 = tk.Radiobutton(frame_encoder, value=1, variable=self.encoder_mode, text='ABI', font=30)
        en_mode1.pack()
        en_mode2.pack()

        self.axis0 = tk.BooleanVar()
        self.axis0.set(True)
        self.axis1 = tk.BooleanVar()
        self.axis1.set(False)
        chk1 = tk.Checkbutton(frame_axis, variable=self.axis0, text='AXIS 0', font=30)
        chk2 = tk.Checkbutton(frame_axis, variable=self.axis1, text='AXIS 1', font=30)
        chk1.pack(expand=1)
        chk2.pack(expand=1)

        self.mode = tk.IntVar()
        self.mode.set(0)
        mode1 = tk.Radiobutton(frame_mode, value=0, variable=self.mode, text='POS', font=30)
        mode2 = tk.Radiobutton(frame_mode, value=1, variable=self.mode, text='VEL', font=30)
        mode3 = tk.Radiobutton(frame_mode, value=2, variable=self.mode, text='TOR', font=30)
        mode1.pack(expand=1)
        mode2.pack(expand=1)
        mode3.pack(expand=1)

        text = tk.StringVar()
        text.set('SET VALUE')
        label = tk.Label(frame_val, textvariable=text, font=30)
        ax0 = tk.Label(frame_val, text='axis 0', font=30)
        ax1 = tk.Label(frame_val, text='axis 1', font=30)
        self.ax0_value = tk.Entry(frame_val, width=5, font=30)
        self.ax1_value = tk.Entry(frame_val, width=5, font=30)
        label.grid(column=0,row=0)
        ax0.grid(column=0,row=1)
        ax1.grid(column=0,row=2)
        self.ax0_value.grid(column=1,row=1)
        self.ax1_value.grid(column=1,row=2)

        idle_btn = tk.Button(frame_motor, text='IDLE', command=self.idle, font=30)
        close_btn = tk.Button(frame_motor, text='CLOSE CONTROL', command=self.close, font=30)
        index_btn = tk.Button(frame_motor, text='INDEX SEARCH', command=self.index, font=30)
        idle_btn.pack(expand=1)
        close_btn.pack(expand=1)
        index_btn.pack(expand=1)

        connect_btn = tk.Button(frame_0, text='CONNECT', command=self.connect, font=30)
        disconnect_btn = tk.Button(frame_0, text='DISCONNECT', command=self.disconnect, font=30)
        calib_btn = tk.Button(frame_btn, text='CALIBRATE', command=self.calib, font=30)
        move_btn = tk.Button(frame_val, text='MOVE', command=self.move, font=30)
        error_btn = tk.Button(frame_val, text='ERROR', command=self.error, font=30)
        quit_btn = tk.Button(frame_btn, text='QUIT', command=self.quit, font=30)
        connect_btn.pack(side=tk.LEFT)
        disconnect_btn.pack(side=tk.LEFT)
        #calib_btn.pack(side=tk.LEFT)
        move_btn.grid(column=2,row=1)
        error_btn.grid(column=2,row=2)
        quit_btn.pack(side=tk.LEFT)

        start_btn = tk.Button(frame_error, text='START', command=self.start_measure, font=30)
        stop_btn = tk.Button(frame_error, text='STOP', command=self.stop_measure, font=30)
        start_btn.pack()
        stop_btn.pack()

        self.fig = plt.figure(figsize=(6,6))
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        self.ax2 = self.fig.add_subplot(2, 1, 2)
        plt.ion()
        self.x = [0] * 20
        self.y = [0] * 20
        self.y1 = [0] * 20
        self.ax1.plot(self.x,self.y)
        self.ax2.plot(self.x,self.y1)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame_right)
        self.plot_widget = self.canvas.get_tk_widget()
        
        self.plot_widget.grid(row=0, column=0)

        frame_left.grid(column=0,row=0)
        frame_right.grid(column=1,row=0)

        frame_1.grid(column=0,row=0)
        frame_0.grid(column=0,row=1)
        frame_2.grid(column=0,row=2)
        frame_val.grid(column=0,row=3)
        frame_error.grid(column=0,row=4)
        frame_btn.grid(column=0,row=5)

        frame_encoder.grid(column=0,row=0,padx=2)
        frame_axis.grid(column=1,row=0,padx=2)

        frame_mode.grid(column=0,row=0,padx=2)
        frame_motor.grid(column=1,row=0,padx=5)

        self.root.mainloop()

    def connect(self):
        self.odrive.connect_test()
        #self.odrive.connect(serial_number=self.serial_number)

    def idle(self):
        if self.axis0.get():
            self.odrive.idle(axis=0)
        if self.axis1.get():
            self.odrive.idle(axis=1)

    def close(self):
        if self.axis0.get():
            self.odrive.engage(axis=0, mode=0)
        if self.axis1.get():
            self.odrive.engage(axis=1, mode=self.encoder_mode.get())

    def index(self):
        if self.axis0.get():
            self.odrive.search(axis=0)
        if self.axis1.get():
            self.odrive.search(axis=1)

    def move(self):
        if self.mode.get() == 0:
            if self.axis0.get():
                self.odrive.control_mode(axis=0, mode="POS")
                self.odrive.drive_pos(axis=0, val=float(self.ax0_value.get()))
            if self.axis1.get():
                self.odrive.control_mode(axis=1, mode="POS")
                self.odrive.drive_pos(axis=1, val=float(self.ax1_value.get()))
        elif self.mode.get() == 1:
            if self.axis0.get():
                self.odrive.control_mode(axis=0, mode="VEL")
                self.odrive.drive_vel(axis=0, val=float(self.ax0_value.get()))
            if self.axis1.get():
                self.odrive.control_mode(axis=1, mode="VEL")
                self.odrive.drive_vel(axis=1, val=float(self.ax1_value.get()))
        elif self.mode.get() == 2:
            if self.axis0.get():
                self.odrive.control_mode(axis=0, mode="TOR")
                self.odrive.drive_tor(axis=0, val=float(self.ax0_value.get()))
            if self.axis1.get():
                self.odrive.control_mode(axis=1, mode="TOR")
                self.odrive.drive_tor(axis=1, val=float(self.ax1_value.get()))

    def disconnect(self):
        self.odrive.disconnect()

    def error(self):
        self.odrive.get_errors()

    def calib(self):
        if self.axis0.get():
            self.odrive.calibrate(axis=0)
        if self.axis1.get():
            self.odrive.calibrate(axis=1)

    def quit(self):
        if self.jobID != None:
            self.root.after_cancel(self.jobID)
        self.root.quit()

    def start_measure(self):
        self.start_time = time.time()
        self.jobID = self.root.after(10,self.measure)

    def stop_measure(self):
        self.stop_time = time.time() - self.start_time
        if self.jobID != None:
            self.root.after_cancel(self.jobID)

    def measure(self):
        self.fig.clf()
        self.x.pop(0)
        self.x.append(time.time()-self.start_time+self.stop_time)
        self.y.pop(0)
        self.y1.pop(0)
        if self.axis0.get():
            self.y.append(self.odrive.get_pos(axis=0))
        else:
            self.y.append(0)
        if self.axis1.get():
            self.y1.append(self.odrive.get_pos(axis=1))
        else:
            self.y1.append(0)
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        self.ax2 = self.fig.add_subplot(2, 1, 2)
        self.ax1.plot(self.x,self.y)
        self.ax2.plot(self.x,self.y1)
        self.fig.canvas.draw()
        self.jobID = self.root.after(10,self.measure)  

def start_odrive():
    #rospy.init_node('odrive')
    #Check_serial_number
    '''
    odrive_node = ODriveNode()
    odrive_node.connect_test() 
    odrive_node.disconnect()
    '''

    odrv_1 = ODriveNode()
    odrv_1.connect(serial_number='207C349F5748')
    #207C34805748
    
    odrv_1.get_errors(mode=1)
    odrv_1.engage()
    odrv_1.get_errors(mode=1) 
    time.sleep(1)
    odrv_1.control_mode(axis=0, mode='VEL')
    odrv_1.drive_vel(axis=0, val=1)
    time.sleep(1)
    odrv_1.drive_vel(axis=0, val=0)
    time.sleep(1)
    odrv_1.get_errors(mode=1)
    odrv_1.disconnect()

def start_odrive_gui():

    odrv_1 = ODriveNode()
    serial_number='207C349F5748'
    gui = Odrive_gui(odrv_1, serial_number)

if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass