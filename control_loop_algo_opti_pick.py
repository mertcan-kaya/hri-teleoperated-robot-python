#!/usr/bin/env python
# Copyright (c) 2020, Universal Robots A/S,
# Modified by Mertcan Kaya (2022-02-14)

import sys
sys.path.append('..')
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

    Pos_ini = np.array([0.2000,-0.2700,0.0400])
    Orn_ini = np.deg2rad(np.array([90,0,0]))

    setlist = [Pos_ini[0], Pos_ini[1], Pos_ini[2], Orn_ini[0], Orn_ini[1], Orn_ini[2]]

    list_to_setp(setp, setlist)

    # deg1 = 23.5928 # asind(0.08535/0.21325)
    # q_pos_ini = np.deg2rad([-90,-180,deg1,180-deg1,-90,90])
    # q_pos = np.deg2rad([-90,-180,deg1,180-deg1,-90,90])

    # arm_len = 0.630 # old: 0.630
    # deg_lim = 45
    # err_coef = 0.25
    # arm_rat = 0.4
    # err_coef = 0.5
    # arm_rat = 0.63
    
    # err_coef_ini = 0.05
    # err_coef_trk = 0.5
    
    # marker_pos = np.zeros(3)
    # marker_pos_prev = np.zeros(3)
    # marker_pos_try = np.zeros(3)

    # # initialize joint pos 
    # setp.input_double_register_0 = q_pos[0]
    # setp.input_double_register_1 = q_pos[1]
    # setp.input_double_register_2 = q_pos[2]
    # setp.input_double_register_3 = q_pos[3]
    # setp.input_double_register_4 = q_pos[4]
    # setp.input_double_register_5 = q_pos[5]

    # the function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
    watchdog.input_int_register_0 = 0

##    print("Testing gripper...")
##    gripper.move_and_wait_for_pos(255, 255, 255)
##    log_info(gripper)
##    gripper.move_and_wait_for_pos(0, 10, 255)
##    log_info(gripper)
            
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
    marker_pos_ini = [-1.569,-0.707,1.337]
    marker_pos = marker_pos_ini
    
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
                marker_pos_try = rot_y(np.deg2rad(-50))@rot_x(np.deg2rad(13.25))@marker_pos_raw
            elif i == 1:
                marker2_pos_raw = np.array(streaming_client.get_marker_pos(1))
                marker2_pos_try = rot_y(np.deg2rad(-50))@rot_x(np.deg2rad(13.25))@marker2_pos_raw

        print(marker_pos_raw)
        print(marker_pos_try)
                
        # do something...
        if state.runtime_state == 2:
            
            if record_data == True:
                now = datetime.now()
                tm_string = now.strftime("%H.%M.%S.%f")
                # write data
                writer.writerow([tm_string, state.actual_q[0], state.actual_q[1], state.actual_q[2], state.actual_q[3], state.actual_q[4], state.actual_q[5],streaming_client.get_marker_pos(0)[0],streaming_client.get_marker_pos(0)[1],streaming_client.get_marker_pos(0)[2]])

            if state.output_int_register_0 == 0:
                
                list_to_setp(setp, setlist)

                # send new setpoint        
                con.send(setp)
                
                #print(marker_pos)
                #print(marker_pos_try)
            else:
                if ini_flag == 0:
                    #print(marker_pos)
                    #print(marker_pos_try)
                    ##print(LA.norm(marker_pos-marker_pos_try))
                    
                    # q_pos = q_pos_ini
                    # err_coef = err_coef_ini
                    
                    ##if LA.norm(marker_pos-marker_pos_try) < 0.05:
                    ##if LA.norm(marker_pos-marker_pos_try) < 0.2:
                    if LA.norm(marker_pos-marker_pos_try) < 0.4:
                        ini_flag = 1
                        #print('GO!')
                else:
                    if LA.norm(marker_pos-marker_pos_try) < 0.1:
                        marker_pos = marker_pos_try
                        # err_coef = err_coef_trk
                
#                     # horizontal
#                     q_pos0_des = q_pos[0]-np.arctan2(marker_pos[1]-marker_pos_ini[1],arm_rat)
#                     q_pos0_fbk = state.actual_q[0]

#                     err0 = q_pos0_des-q_pos0_fbk

#                     #q_pos[0] = np.deg2rad(q_pos_ini[0])+0.25*err0
#                     q_pos[0] = np.deg2rad(-90)+err_coef*err0

#                     if np.abs(err0) > np.deg2rad(10):
#                         q_pos0_des = q_pos0_fbk
                    
# ##                    if q_pos0_fbk < np.deg2rad(q_pos_ini[0]-deg_lim):
# ##                        q_pos[0] = np.deg2rad(q_pos_ini[0]-deg_lim)
#                     if q_pos0_fbk < np.deg2rad(-90-deg_lim):
#                         q_pos[0] = np.deg2rad(-90-deg_lim)

# ##                    if q_pos0_fbk > np.deg2rad(q_pos_ini[0]+deg_lim):
# ##                        q_pos[0] = np.deg2rad(q_pos_ini[0]+deg_lim)
#                     if q_pos0_fbk > np.deg2rad(-90+deg_lim):
#                         q_pos[0] = np.deg2rad(-90+deg_lim)

#                     # vertical
#                     q_pos1_des = q_pos[1]-np.arctan2(marker_pos[0]-marker_pos_ini[0],arm_rat)
#                     q_pos1_fbk = state.actual_q[1]
                    
#                     err1 = q_pos1_des-q_pos1_fbk
                    
#                     #q_pos[1] = np.deg2rad(q_pos_ini[1])-0.25*err1
#                     q_pos[1] = np.deg2rad(-180)-err_coef*err1
                    
#                     if np.abs(err1) > np.deg2rad(10):
#                         q_pos1_des = q_pos1_fbk
                    
# ##                    if q_pos1_fbk < np.deg2rad(q_pos_ini[1]-deg_lim):
# ##                        q_pos[1] = np.deg2rad(q_pos_ini[1]-deg_lim)
#                     if q_pos1_fbk < np.deg2rad(-180-deg_lim):
#                         q_pos[1] = np.deg2rad(-180-deg_lim)
                        
# ##                    if q_pos1_fbk > np.deg2rad(q_pos_ini[1]+deg_lim):
# ##                        q_pos[1] = np.deg2rad(q_pos_ini[1]+deg_lim)
#                     if q_pos1_fbk > np.deg2rad(-180+deg_lim):
#                         q_pos[1] = np.deg2rad(-180+deg_lim)

                #setlist = [q_pos[0], q_pos[1], q_pos[2], q_pos[3], q_pos[4], q_pos[5]]

##                if q_pos[0] < np.deg2rad(-80):
##                    gripper.move(255, 255, 255)
##                else:
##                    gripper.move(0, 255, 255)
                
                if streaming_client.get_marker_count() > 1:                    
                    #print("\t")
                    #print(marker2_pos_try)
                    dist_btw_points = np.sqrt(pow(marker2_pos_try[0]-marker_pos_try[0],2)+pow(marker2_pos_try[1]-marker_pos_try[1],2)+pow(marker2_pos_try[2]-marker_pos_try[2],2))
                    #print(dist_btw_points)

                if dist_btw_points < 0.045:
                    closed = 1
                else:
                    closed = 0

                #print(closed)

                if closed == 1:
                    grip_rat = 0
                else:
                    if dist_btw_points > 0.1:
                        grip_rat = 100
                    else:
                        grip_rat = (dist_btw_points - 0.045) * 100 / (0.1 - 0.045)
                
                #print(grip_rat)

                grip_pos = 255 - int(grip_rat * 2.55)
                
                #print(grip_pos)
                
                gripper.move(grip_pos, 255, 255)
                
                list_to_setp(setp, setlist)
                # send new setpoint        
                con.send(setp)
        ##else:
            ##marker_pos_ini = marker_pos_try
            
        # kick watchdog
        con.send(watchdog)

    if record_data == True:
        # close write file
        outfile.close()
    
    print('Disconnected')

    con.send_pause()
    con.disconnect()
