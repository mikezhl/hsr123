#!/usr/bin/env python
import numpy as np
import sys
import pyexotica as exo

import rospy
from geometry_msgs.msg import PoseStamped
import hsrb_interface

import my_arm_client as arm
import my_base_client as base
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
    KDLFrame_goal = KDLFrame_base.inverse()* KDLFrame_traj
    return KDLFrame_goal.get_translation_and_rpy()

traj_rrt1 = np.load(sys.path[0]+"/pickup_traj/"+"traj_rrt1.npy")
traj_rrt2 = np.load(sys.path[0]+"/pickup_traj/"+"traj_rrt2.npy")
traj_aico = np.load(sys.path[0]+"/pickup_traj/"+"traj_aico.npy")
dt=1
vel_limit = 0.1
start=[-1,-2,0.70558]
# Initialize
try:
    robot = hsrb_interface.Robot()
    whole_body = robot.get('whole_body')
    hsrb_gripper = robot.get('gripper')
    open_gripper(hsrb_gripper)
except:
    raise Exception("Fail to initialize")

cli_arm, cli_base = my_Simple_Action_Clients()
    
# Find the current position
KDLFrame_startbase = exo.KDLFrame([start[0],start[1],0,0,0,start[2]])
base_list = np.array([transform_base_traj(traj_rrt1[i], KDLFrame_startbase) for i in range(len(traj_rrt1))])
col_index = [0,1,5]
base_list = base_list[:, col_index]


# Get user input to run traj in gazebo
print("loading base goal")
base.load_base_goal_rrt(base_list,cli_base,vel_limit)

while not rospy.is_shutdown():
        rospy.spin()
print('All Done :)')





