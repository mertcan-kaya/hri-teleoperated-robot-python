#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# Modified by Mertcan Kaya (2023-07-25)

import sys

sys.path.append("..")
import logging

import os
from datetime import datetime
import csv
import trajectory as trj
import numpy as np
from numpy import linalg as LA
import subprocess as sp

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import time
from NatNetClient import NatNetClient

import robotiq_gripper

mount_config = 0 # 0: shoulder, 1: table

record_data = False

def rot_y(rad):
    return np.array([[np.cos(rad),0.,np.sin(rad)],[0.,1.,0.],[-np.sin(rad),0.,np.cos(rad)]])

def rot_z(rad):
    return np.array([[np.cos(rad),-np.sin(rad),0.],[np.sin(rad),np.cos(rad),0.],[0.,0.,1.]])

def rot_x(rad):
    return np.array([[1.,0.,0.],[0.,np.cos(rad),-np.sin(rad)],[0.,np.sin(rad),np.cos(rad)]])

# functions
def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range(0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

def print_list(plist):
    for i in range(0,len(plist)):
        if i < len(plist)-1:
            print("%7.4f" % plist[i], end=", ")
        else:
            print("%7.4f" % plist[i], end="\n")

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict

if __name__ == "__main__":

    # OptiTrack initialization
    
    optionsDict = {}
    optionsDict["clientAddress"] = "127.0.0.1"
    optionsDict["serverAddress"] = "127.0.0.1"
    optionsDict["use_multicast"] = True

    # This will create a new NatNet client
    optionsDict = my_parse_args(sys.argv, optionsDict)
    print(optionsDict)
    
    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    streaming_client.set_print_level(1)
    
    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")
    
    # configure recipes (host, port)
    ROBOT_HOST = '10.8.0.231'#'localhost' 
    ROBOT_PORT = 30004
    
    ## robotiq gripper
    def log_info(gripper):
        print(f"Pos: {str(gripper.get_current_position()): >3}  "
              f"Open: {gripper.is_open(): <2}  "
              f"Closed: {gripper.is_closed(): <2}  ")

    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ROBOT_HOST, 63352)
    print("Activating gripper...")
    gripper.activate()

    # configure recipes
    config_filename = 'control_loop_configuration.xml'
    
    #logging.basicConfig(level=logging.INFO)
    logging.getLogger().setLevel(logging.INFO)

    conf = rtde_config.ConfigFile(config_filename)
    state_names, state_types = conf.get_recipe('state')
    setp_names, setp_types = conf.get_recipe('setp')
    command_names, command_types = conf.get_recipe('command')
    watchdog_names, watchdog_types = conf.get_recipe('watchdog')

    # connect to controller
    con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
    con.connect()

    # get controller version
    con.get_controller_version()

    print('Connected to robot')
    
    # setup recipes
    con.send_output_setup(state_names, state_types)
    setp = con.send_input_setup(setp_names, setp_types)
    command = con.send_input_setup(command_names, command_types)
    watchdog = con.send_input_setup(watchdog_names, watchdog_types)
    
    if record_data == True:
        # detect the current working directory and print it
        patha = os.getcwd()
        print ("The current working directory is %s" % patha)

        # define the name of the directory to be created
        save_path = "recorded data"
        pathb = patha + '/' + save_path
        try:
            os.mkdir(pathb)
        except OSError:
            print ("Creation of the directory %s failed" % pathb)
        else:
            print ("Successfully created the directory %s " % pathb)    
        now = datetime.now()
        d_string = now.strftime("%Y-%m-%d %H.%M.%S")
        filename = 'data ' + d_string + '.csv'
        completeName = save_path + '\\' + filename
        
        # log actual joint pos
        outfile = open(completeName, 'w', newline='')
        writer = csv.writer(outfile, delimiter=',')
        list_time = ['time']
        list_actual_q = ['actual_q0', 'actual_q1', 'actual_q2', 'actual_q3', 'actual_q4', 'actual_q5']
        list_marker_pos = ['marker_pos_x', 'marker_pos_y', 'marker_pos_z']
        writer.writerow(list_time+list_actual_q+list_marker_pos)
    
    # initial pos

    ## Enum

    # mounting configuration
    SHLDRM = 0 # shoulder mount
    TABLEM = 1 # table mount
    
    if mount_config == 1:
        mounting_seq = TABLEM
    else:
        mounting_seq = SHLDRM

    if mounting_seq == SHLDRM:
        # Shoulder
        Pos_ini = np.array([0.2000,-0.2500,0.5400])
        #Orn_ini = np.deg2rad(np.array([0,90,0]))
        Orn_ini = np.array([1.199626905,-1.207793505,1.204287902])
    else:
        # Table
        Pos_ini = np.array([0.2000,-0.2700,0.0400])
        Orn_ini = np.deg2rad(np.array([90,0,0]))

    setlist = [Pos_ini[0], Pos_ini[1], Pos_ini[2], Orn_ini[0], Orn_ini[1], Orn_ini[2]]

    list_to_setp(setp, setlist)

    # the function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
    watchdog.input_int_register_0 = 0

    # start data synchronization
    if not con.send_start():
        sys.exit()

    # joint_space:0, task_space:1
    command.input_int_register_1 = 1

    # init_joint:0, init_task:1
    command.input_int_register_2 = 1
            
    # trajectory command
    command.input_int_register_3 = 1
    
    con.send(command)

    print('Press play to start')

    keep_running = True
    
    ini_flag = 0
    
    #marker_pos_ini = [0.216,-0.930,1.538]
    marker_pos_ini = [-0.142,-0.180,1.792]
    marker_pos = marker_pos_ini
    marker_pos_zero = [0.0,0.0,0.0]
    marker_pos_robot = np.array([0.0,0.0,0.0])

    Pos_x = Pos_ini[0]
    Pos_y = Pos_ini[1]
    Pos_z = Pos_ini[2]

    marker_pos_try = np.zeros(3)
    marker2_pos_try = [0.0,0.0,0.0]
    dist_btw_points = 1
    closed = 0

    idx = 1
    # control loop
    while keep_running:
        # receive the current state
        state = con.receive()
        
        if state is None:
            break
        
        for i in range(0,streaming_client.get_marker_count()):
            if i == 0:
                marker_pos_raw = np.array(streaming_client.get_marker_pos(0))
                marker_pos_try = rot_y(np.deg2rad(0))@rot_x(np.deg2rad(28))@marker_pos_raw
            elif i == 1:
                marker2_pos_raw = np.array(streaming_client.get_marker_pos(1))
                marker2_pos_try = rot_y(np.deg2rad(0))@rot_x(np.deg2rad(28))@marker2_pos_raw

        # do something...
        if state.runtime_state == 2: # Play button is pressed
            
            if state.output_int_register_0 == 0: # Continue button is pressed
                ini_flag = 0
            else:
                #print('play')
                idx = idx + 1
                if ini_flag == 0:
                    if LA.norm(marker_pos-marker_pos_try) < 0.1:
                        ini_flag = 1
                        print('GO!')
                else:
                    if LA.norm(marker_pos-marker_pos_try) < 0.1:
                        marker_pos = marker_pos_try
                        # err_coef = err_coef_trk
                        
                    marker_pos_zero = np.array(marker_pos)-np.array(marker_pos_ini)

                    if mounting_seq == SHLDRM:
                        Pos_x = Pos_ini[0] - marker_pos_zero[1]
                        Pos_y = Pos_ini[1] + marker_pos_zero[0]
                        Pos_z = Pos_ini[2] - marker_pos_zero[2]
                    else:
                        Pos_x = Pos_ini[0] - marker_pos_zero[2] # left-right
                        Pos_y = Pos_ini[1] + marker_pos_zero[0] # back-forward
                        Pos_z = Pos_ini[2] + marker_pos_zero[1] # up-down

            if mounting_seq == SHLDRM:
                if Pos_x > 0.2: # bottom limit
                    Pos_x = 0.2

                if Pos_x < -0.125: # top limit
                    Pos_x = -0.125
                
                if Pos_z > 0.56: # right limit
                    Pos_z = 0.56
                    
                if Pos_z < 0.15 and Pos_x <= -0.075: # left limit on top part
                    Pos_z = 0.15

                if Pos_z < 0.47 and Pos_x > -0.075: # left limit on bottom part
                    Pos_z = 0.47
                    
                if Pos_y < -0.30 and Pos_z > 0.53: # singularity avoidance
                    Pos_z = 0.53
                    
                if Pos_y > -0.25: # back limit
                    Pos_y = -0.25

                if Pos_y < -0.37: # forward limit
                    Pos_y = -0.37
                    
            else: # Table
                
                if Pos_z < Pos_ini[2]: # down limit
                    Pos_z = 0.04
                    
                if Pos_z > Pos_ini[2] + 0.325: # up limit
                    Pos_z = 0.365
                    
                if Pos_y > Pos_ini[1]: # back limit
                    Pos_y = -0.27

                if Pos_y < Pos_ini[1] - 0.12: #forward limit
                    Pos_y = -0.39
                
                if Pos_x > Pos_ini[0] + 0.05: # right limit
                    Pos_x = 0.225

                if Pos_x < Pos_ini[0] - 0.1 and Pos_z <= 0.275 : # left limit on bottom part
                    Pos_x = Pos_ini[0] - 0.1
                
                if Pos_x < Pos_ini[0] - 0.3 and Pos_z > 0.275 : # left limit on top part
                    Pos_x = Pos_ini[0] - 0.3
                
            setlist = [Pos_x, Pos_y, Pos_z, Orn_ini[0], Orn_ini[1], Orn_ini[2]]
                
            if streaming_client.get_marker_count() > 1:
                dist_btw_points = np.sqrt(pow(marker2_pos_try[0]-marker_pos_try[0],2)+pow(marker2_pos_try[1]-marker_pos_try[1],2)+pow(marker2_pos_try[2]-marker_pos_try[2],2))
            
            if dist_btw_points < 0.045:
                closed = 1
            else:
                closed = 0

            if closed == 1:
                grip_rat = 0
            else:
                if dist_btw_points > 0.1:
                    grip_rat = 100
                else:
                    grip_rat = (dist_btw_points - 0.045) * 100 / (0.1 - 0.045)
            
            grip_pos = 255 - int(grip_rat * 2.55)
            
            gripper.move(grip_pos, 255, 255)
                
            list_to_setp(setp, setlist)
            # send new setpoint        
            con.send(setp)
        
        # kick watchdog
        con.send(watchdog)

    if record_data == True:
        # close write file
        outfile.close()
    
    print('Disconnected')
    con.send_pause()
    con.disconnect()
