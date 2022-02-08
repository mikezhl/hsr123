#!/usr/bin/env python
import numpy as np
import sys
import pyexotica as exo
from geometry_msgs.msg import PoseStamped
import rospy
import hsrb_interface

import my_arm_client as arm
import my_base_client as base
from my_functions import my_Simple_Action_Clients, my_traj_transform

traj_rrt1 = np.load(sys.path[0]+"/pickup_traj/"+"traj_rrt1.npy")
traj_rrt2 = np.load(sys.path[0]+"/pickup_traj/"+"traj_rrt2.npy")
traj_aico = np.load(sys.path[0]+"/pickup_traj/"+"traj_aico.npy")
dt=0.15
vel_limit = 0.1
start=[-1,-2,0.70558]
# Initialize
try:
    robot = hsrb_interface.Robot()
    whole_body = robot.get('whole_body')
    hsrb_gripper = robot.get('gripper')
    hsrb_gripper.command(0.8)
except:
    raise Exception("Fail to initialize")

cli_arm, cli_base = my_Simple_Action_Clients()

# Constrct base list of rrt part1
print("rrt part1")
KDLFrame_startbase = exo.KDLFrame([start[0],start[1],0,0,0,start[2]])
base_list = np.array([my_traj_transform(traj_rrt1[i], KDLFrame_startbase) for i in range(len(traj_rrt1))])
p_base_list_1 = base.prepare_rrt(base_list,vel_limit)
# Constrct base list of aico part
print("aico part")
time_list = np.arange(0.0,dt*len(traj_aico),dt)
arm_list = traj_aico[:,3:8]
arm_list = np.c_[arm_list,time_list]
arm_list = arm_list[1::]
p_arm_list1 = arm.prepare_aico(arm_list,dt)

traj_aico_base = traj_aico[:,0:3]
base_list = np.array([my_traj_transform(traj_aico_base[i], KDLFrame_startbase) for i in range(len(traj_aico_base))])
base_list = np.c_[base_list, time_list]
base_list = base_list[1::]
p_base_list_2 = base.prepare_aico(base_list,dt)
# Constrct base list of rrt part2
print("rrt part2")
base_list = np.array([my_traj_transform(traj_rrt2[i], KDLFrame_startbase) for i in range(len(traj_rrt2))])
p_base_list_3 = base.prepare_rrt(base_list,vel_limit)

# Run!!
rospy.set_param('pickup_status', 0)
print("loading base goal1")
base.load_base_goal_pickup(p_base_list_1,cli_base)
while rospy.get_param("pickup_status")==0:
    rospy.sleep(0.1)

print("loading base goal2")
base.load_base_goal_pickup(p_base_list_2,cli_base)
arm.load_arm_goal_pickup(p_arm_list1,cli_arm)
rospy.sleep(4*dt/0.1)
hsrb_gripper.apply_force(0.2, delicate = True)
while rospy.get_param("pickup_status")<3:
    rospy.sleep(0.1)

print("loading base goal3")
base.load_base_goal_pickup(p_base_list_3,cli_base)
while rospy.get_param("pickup_status")==3:
    rospy.sleep(0.1)

print('All Done :)')





