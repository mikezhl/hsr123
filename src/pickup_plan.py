from pickup_ik import pickup_ik
from pickup_rrt import pickup_rrt
from pickup_aico import pickup_aico
from my_functions import my_transform_can, my_plot_analysis, my_get_pose, my_pickup,find_aico_point,my_bezier,my_set_traj
import rospy
import matplotlib.pyplot as plt
import numpy as np
import sys
import math
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

# Init
debug=1
start=[-1,-2,0]
end=[3,-1,0]
can_position = [0.8856, -0.3937, 0.7941]
# For setting orientation
Y_r2c = math.atan2(can_position[1]-start[1],can_position[0]-start[0])
Y_c2r = math.atan2(start[1]-can_position[1],start[0]-can_position[0])
start[2] = Y_r2c
# Find the start and end for AICO
base_for_pickup = pickup_ik(can_position,debug=0)
print("base_for_pickup: ",base_for_pickup[0][0:3])
traj_rrt1_full = pickup_rrt(start,base_for_pickup[0][0:3],debug=0,doplot=0)
traj_rrt2_full = pickup_rrt(base_for_pickup[0][0:3],end,debug=0,doplot=0)
traj_rrt1,traj_rrt2 = find_aico_point(traj_rrt1_full,traj_rrt2_full,0.25,can_position,1)
aico_start,aico_end = traj_rrt1[-1,:],traj_rrt2[0,:]
print("AICO start and end: ", aico_start,aico_end)
# Generate the base traj for AICO
traj_path="/home/hzhu/ros_123/src/hsr123/resources/pickup/base.traj"
if (sum((aico_start-aico_end)**2))**0.5<1.2:
    x,y,theta = my_bezier(can_position,[aico_start,aico_end],20,debug=0)
else:
    x=[aico_start[0],aico_end[0]]
    y=[aico_start[1],aico_end[1]]
    theta=[aico_start[2],aico_end[2]]
my_set_traj(x,y,theta,traj_path)
if debug:
    # Plot the base traj
    plt.plot(traj_rrt1_full[:,0],traj_rrt1_full[:,1],'xr')
    plt.plot(traj_rrt2_full[:,0],traj_rrt2_full[:,1],'xb')
    plt.plot(can_position[0],can_position[1],'og')
    plt.plot(x,y,'k')
    circle1 = plt.Circle((can_position[0],can_position[1]), 1, color='r',fill=False)
    plt.gca().add_patch(circle1)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
# Generate traj for grasping
traj_aico = pickup_aico(can_position,debug=0,doplot=0)
if debug:
    # Move in RViz
    np.save(sys.path[0]+"/pickup_traj/"+"traj_rrt1",traj_rrt1)
    np.save(sys.path[0]+"/pickup_traj/"+"traj_rrt2",traj_rrt2)
    np.save(sys.path[0]+"/pickup_traj/"+"traj_aico",traj_aico)
    print(traj_rrt1,"\n",traj_aico,"\n",traj_rrt2)
    arm_traj = [0,0,-np.pi/2,-np.pi/2,0]
    traj_rrt1 = np.concatenate((traj_rrt1,np.resize(arm_traj,(len(traj_rrt1),5))), axis=1)
    traj_rrt2 = np.concatenate((traj_rrt2,np.resize(arm_traj,(len(traj_rrt2),5))), axis=1)
    my_pickup(np.concatenate((traj_rrt1,traj_aico,traj_rrt2), axis=0),can_position)