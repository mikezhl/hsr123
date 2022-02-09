from pickup_ik import pickup_ik
from pickup_rrt import pickup_rrt
from pickup_aico import pickup_aico
from my_functions import my_transform_can, my_plot_analysis, my_get_pose, my_pickup,find_aico_point,my_aico_traj_new
import rospy
import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# Init
debug=1
start=[-1,-2,0]
end=[3,-1,0]
can_position = [0.8856, -0.3937, 0.7941]
place_position = [0.9, -1, 1]
# For setting orientation
Y_r2c = math.atan2(can_position[1]-start[1],can_position[0]-start[0])
Y_c2r = math.atan2(start[1]-can_position[1],start[0]-can_position[0])
start[2] = Y_r2c

# Find the start and end for AICO pickup
# base_for_pickup = pickup_ik(can_position,debug=0)
# base_for_place = pickup_ik(place_position,debug=0)
base_for_pickup = [[0.8263, 0,  -0.9085]]
base_for_place = [[1.0022, -1.4,  2.4842]]
print("base_for_pickup: ",base_for_pickup[0][0:3])
print("base_for_place: ",base_for_place[0][0:3])
traj_rrt1_full = pickup_rrt(start,base_for_pickup[0][0:3],debug=0,doplot=0)
traj_rrt2_full = pickup_rrt(base_for_pickup[0][0:3],base_for_place[0][0:3],debug=0,doplot=0)
traj_rrt3_full = pickup_rrt(base_for_place[0][0:3],end,debug=0,doplot=0)
traj_rrt1,traj_rrt21 = find_aico_point(traj_rrt1_full,traj_rrt2_full,0.25,base_for_pickup[0],0.5)
traj_rrt2,traj_rrt3 = find_aico_point(traj_rrt21,traj_rrt3_full,0.25,base_for_place[0],0.5)
aico_start1,aico_end1 = traj_rrt1[-1,:],traj_rrt21[0,:]
aico_start2,aico_end2 = traj_rrt2[-1,:],traj_rrt3[0,:]
print("AICO start1 and end1: ", aico_start1,aico_end1)
print("AICO start2 and end2: ", aico_start2,aico_end2)

# Generate the base traj for AICO
traj_path1="/home/hzhu/ros_123/src/hsr123/resources/pickup/base1.traj"
traj_path2="/home/hzhu/ros_123/src/hsr123/resources/pickup/base2.traj"
x1,y1 = my_aico_traj_new(aico_start1,aico_end1,base_for_pickup[0],20,traj_path1)
x2,y2 = my_aico_traj_new(aico_start2,aico_end2,base_for_place[0],20,traj_path2)

# Plot the base traj
if debug:
    plt.plot(traj_rrt1_full[:,0],traj_rrt1_full[:,1],'xr')
    plt.plot(traj_rrt2_full[:,0],traj_rrt2_full[:,1],'xg')
    plt.plot(traj_rrt3_full[:,0],traj_rrt3_full[:,1],'xb')
    plt.plot(can_position[0],can_position[1],'og')
    plt.plot(place_position[0],place_position[1],'og')
    circle1 = plt.Circle((base_for_pickup[0][0],base_for_pickup[0][1]), 0.5, color='r',fill=False)
    circle2 = plt.Circle((base_for_place[0][0],base_for_place[0][1]), 0.5, color='r',fill=False)
    plt.gca().add_patch(circle1)
    plt.gca().add_patch(circle2)
    color1 = [str(i/len(x1)) for i in range(len(x1))]
    color2 = [str(i/len(x2)) for i in range(len(x2))]
    plt.scatter(x1,y1,s=100,c=color1)
    plt.scatter(x2,y2,s=100,c=color2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
# Generate traj for grasping
traj_aico1 = pickup_aico(can_position,traj_path1,debug=0,doplot=0)
traj_aico2 = pickup_aico(place_position,traj_path2,debug=0,doplot=0)
if debug:
    # Move in RViz
    np.save(sys.path[0]+"/pickup_traj/"+"traj_rrt1",traj_rrt1)
    np.save(sys.path[0]+"/pickup_traj/"+"traj_rrt2",traj_rrt2)
    np.save(sys.path[0]+"/pickup_traj/"+"traj_rrt3",traj_rrt3)
    np.save(sys.path[0]+"/pickup_traj/"+"traj_aico1",traj_aico1)
    np.save(sys.path[0]+"/pickup_traj/"+"traj_aico2",traj_aico2)
    arm_traj = [0,0,-np.pi/2,-np.pi/2,0]
    traj_rrt1 = np.concatenate((traj_rrt1,np.resize(arm_traj,(len(traj_rrt1),5))), axis=1)
    traj_rrt2 = np.concatenate((traj_rrt2,np.resize(arm_traj,(len(traj_rrt2),5))), axis=1)
    traj_rrt3 = np.concatenate((traj_rrt3,np.resize(arm_traj,(len(traj_rrt3),5))), axis=1)
    my_pickup(np.concatenate((traj_rrt1,traj_aico1,traj_rrt2,traj_aico2,traj_rrt3), axis=0),can_position)