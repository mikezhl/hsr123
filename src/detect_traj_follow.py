#!/usr/bin/env python
import numpy as np
import sys
import pyexotica as exo

import rospy
from geometry_msgs.msg import PoseStamped
import hsrb_interface

import my_arm_client as arm
import my_base_client as base
from detect_traj_aico import detect_traj_aico
from my_functions import my_Simple_Action_Clients

def open_gripper(hsrb_gripper):
    # Approximate radians of open gripper
    _GRASP_OPEN = 0.8
    hsrb_gripper.command(_GRASP_OPEN)

def close_gripper(hsrb_gripper):
    # Grasp force[N]
    _GRASP_FORCE=0.2
    hsrb_gripper.apply_force(_GRASP_FORCE, delicate = True)

def transform_base_traj(base_list, KDLFrame_base):
    '''Takes input of base_pose which is an array [x,y,rot] and a KDLFrame.
        returns [x,y,z,r,p,yaw]
    '''
    KDLFrame_traj = exo.KDLFrame(np.concatenate((base_list[0:2],[0,0,0],[base_list[2]])))
    KDLFrame_goal = KDLFrame_base * KDLFrame_traj
    return KDLFrame_traj.get_translation_and_rpy()

def detect_traj_follow(grasp_t, dt, *tag_pose, new_traj=0):
    '''time for grasping, time step of trajectory in seconds, new_traj(0=use a generated traj;1=generate a new traj), pose of tag].
    '''
    # Initialize
    try:
        robot = hsrb_interface.Robot()
        whole_body = robot.get('whole_body')
        hsrb_gripper = robot.get('gripper')
        open_gripper(hsrb_gripper)
        whole_body.move_to_go()
    except:
        raise Exception("Fail to initialize")
    
    # Get trajectory
    if new_traj:
        received_traj = detect_traj_aico(debug=0,tag_pose=tag_pose[0],t_grasp_begin=grasp_t,doplot=0)
    else:
        # received_traj = np.load(sys.path[0]+'/trajectories/rob.npy', allow_pickle = False)
        received_traj = np.load(sys.path[0]+'/trajectories/detect.npy', allow_pickle = False)
    print("The shape of the trajectory:",np.shape(received_traj))

    # Create Simple Action Clients for arm and base
    cli_arm, cli_base = my_Simple_Action_Clients()
    
    # Find the current position
    base_posestamped = rospy.wait_for_message('/global_pose', PoseStamped)
    base_pose = base_posestamped.pose
    x = base_pose.position.x
    y = base_pose.position.y
    z = base_pose.position.z
    qx= base_pose.orientation.x
    qy= base_pose.orientation.y
    qz= base_pose.orientation.z
    qw= base_pose.orientation.w
    KDLFrame_base = exo.KDLFrame([x,y,z,qx,qy,qz,qw])

    time_list = np.arange(0.0,dt*len(received_traj),dt)

    # Formulate trajectories - arm
    arm_list = received_traj[:,3:8]
    arm_list = np.c_[arm_list,time_list]
    
    # Formulate trajectories - base
    base_list = received_traj[:,0:3]
    # base_list starts from 0,0,0.
    base_list = np.array([transform_base_traj(base_list[i], KDLFrame_base) for i in range(len(base_list))])
    # Index of x,y,yaw
    col_index = [0,1,5]
    base_list = base_list[:, col_index]
    base_list = np.c_[base_list, time_list]

    # Remove the zero-th time step because it has an undefined velocity
    base_list = base_list[1::]
    arm_list = arm_list[1::]

    # Get user input to run traj in gazebo
    print("press enter to continue")
    input()
    print("loading arm goal")
    arm.load_arm_goal(arm_list,cli_arm,dt)
    print("loading base goal")
    base.load_base_goal(base_list,cli_base,dt)

    # Gripper controller doesnt work. not listed in lise_controllers.
    # Thus jank stuff with rospy sleep to get the gripper to close at the right time
    open_gripper(hsrb_gripper)
    print("Gripper sleeping")
    rospy.sleep(grasp_t*dt/0.1)
    print("Gripper closing")
    close_gripper(hsrb_gripper)
    while not rospy.is_shutdown():
        rospy.spin()
    print('All Done :)')
    exit()

if __name__ == '__main__':
    # Set tag_pose if new_traj=1
    detect_traj_follow(4.5, 0.15, new_traj=0)
    # detect_traj_follow(4.5, 0.15, 2.5, new_traj=1)

