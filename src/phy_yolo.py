import numpy as np
import math
import sys
import hsrb_interface
from phy_detect import phy_detect
import pyexotica as exo
import threading
import pickle
import time
from pickup_ik import pickup_ik
from pickup_rrt import pickup_rrt_loop
from pickup_aico import pickup_aico
from image.detect import detect_all
from my_functions import *
import my_arm_client as arm
import my_base_client as base

# Settings
gazebo=1
dt=0.5
vel_limit = 0.05
v_max = 0.2
acceleration=0.05
rospy.init_node("phy_yolo")
spawn_position = [0,0,0]
if gazebo:
    print("Initializing")
    robot = hsrb_interface.Robot()
    # omni_base = robot.get('omni_base')
    # omni_base.go_abs(0,0,0,0)
    print("Finding position")
    # start_position = my_get_position(spawn_position)
    start_position=[0,0,0]
    # print(start_position)
    try:
        print("Getting whole_body")
        whole_body = robot.get('whole_body')
        print("Getting gripper")
        hsrb_gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
        print("Openning griper")
        hsrb_gripper.command(0.8)
    except:
        raise Exception("Fail to initialize")
    print("Getting action clients")
    cli_arm, cli_base = my_Simple_Action_Clients()
    client_all = [cli_arm, cli_base, whole_body, hsrb_gripper]
    can_position_list = phy_detect()
    can_position_list = [my_transform_can_yolo_list(i,spawn_position) for i in can_position_list]
    print(can_position_list)
    # can_position_list = [[1.2348730948876068, -0.06301005305985588, 0.7400708592897839]]
else:
    start_position = [0,0,0]
    can_position_list = [[1.1853382256035114, 0.006000162919329144, 0.8158672610935684]]

end_position = [0,-1,0]
scene_list = ["{hsr123}/resources/phy_yolo_table.scene","{hsr123}/resources/soda_can.scene"]


print("Planning")
base_for_pickup = pickup_ik(can_position_list[0],scene_list,debug=0)
traj_path1=sys.path[0]+"/pickup_traj/phy_yolo.traj"
x1,y1 = my_aico_traj_new(start_position,end_position,base_for_pickup,20,traj_path1)

# Plot the base trajectory
plt.plot(base_for_pickup[0],base_for_pickup[1],'xr')
rt1 = plt.Rectangle((1.15,-0.7),0.7,1.4)
plt.gca().add_patch(rt1)
color1 = np.array([[i/(len(x1)-1)*(255-128)+128,0,i/(len(x1)-1)*(255-128)+128] for i in range(len(x1))])/255
plt.scatter(x1,y1,s=100,c=color1)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

traj_aico1 = pickup_aico(can_position_list[0],traj_path1,scene_list,gripper_orientation=0,debug=0,doplot=0)

# Visualization in rviz
if 1-gazebo:
    my_pickup(traj_aico1,can_position_list[0],scene_list)

print("Constrcting TrajectoryPoint list")
time_list = np.arange(0.0,dt*len(traj_aico1),dt)
arm_list = traj_aico1[:,3:8]
arm_list = np.c_[arm_list,time_list]
arm_list = arm_list[1::]
p_arm_list1 = arm.prepare_aico(arm_list,dt)
traj_aico_base = traj_aico1[:,0:3]
KDLFrame_startbase = exo.KDLFrame([spawn_position[0],spawn_position[1],0,0,0,spawn_position[2]])
base_list = np.array([my_traj_transform(traj_aico_base[i], KDLFrame_startbase) for i in range(len(traj_aico_base))])
base_list = np.c_[base_list, time_list]
base_list = base_list[1::]
p_base_list_2 = base.prepare_aico(base_list,dt)

print("Loading base goal2")
base.load_base_goal_pickup(p_base_list_2,cli_base)
arm.load_arm_goal_pickup(p_arm_list1,cli_arm)
rospy.sleep(4*dt/0.1)
hsrb_gripper.apply_force(1, delicate = True)
