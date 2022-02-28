from cProfile import label
import time
import numpy as np
import rospy
import math
import sys
import pyexotica as exo
import pickle
import trajectory_msgs.msg
from my_functions import my_traj_transform
import matplotlib.pyplot as plt

def base_point(current_base_traj, current_base_velocity):
    p_base = trajectory_msgs.msg.JointTrajectoryPoint()
    p_base.velocities = current_base_velocity[0:3]
    p_base.positions = current_base_traj[0:3]
    p_base.time_from_start = rospy.Time(current_base_traj[3])
    return p_base
def prepare_rrt(base_list,v_max,acceleration):
    '''base_list in the form of [base angle x3]'''
    distance = np.diff(base_list, axis = 0)
    distance_abs = (distance[:,0]**2+distance[:,1]**2)**0.5
    cum=np.cumsum(distance_abs)
    time_list = np.zeros(len(cum))
    t1 = v_max/acceleration
    s1 = 0.5*v_max*t1
    s2 = cum[-1]-s1
    t2 = (s2-s1)/v_max+t1
    t3 = cum[-1]/v_max+v_max/acceleration
    s3 = cum[-1]
    time_num = 200000
    small_time_list = np.linspace(0,t1,time_num)
    small_dis_list = small_time_list/2*(v_max+(v_max-acceleration*small_time_list))
    for i in range(len(cum)):
        if cum[i] <= s1:
            time_list[i] = (cum[i]*2/acceleration)**0.5
        elif cum[i] >= cum[-1]-s1 and cum[i]<=s3:
            ss = cum[i]-s2
            tt = min(range(len(small_dis_list)), key=lambda i: abs(small_dis_list[i]-ss))
            time_list[i] = tt/time_num*t1+t2
        else:
            time_list[i] = (cum[i]-s1)/v_max+t1
    # time_list = (time_list+0.5)*1.5
    base_list = np.c_[base_list, np.concatenate(([0],time_list))]
    base_vel  = [(base_list[i+1,0:3]-base_list[i,0:3])/(base_list[i+1,3]-base_list[i,3]) for i in range(len(base_list)-1)]
    base_list = base_list[1::]
    base_vel=np.array(base_vel)
    print(base_vel[0])
    plt.plot(time_list,(base_vel[:,0]**2+base_vel[:,1]**2)**0.5,"r",label="velocity-now")
    plt.plot(time_list,cum,"b",label="distance-now")
    # time_before = cum[-1]/0.03
    # plt.plot([0,time_before],[0.03,0.03],"r--",label="velocity-before")
    # plt.plot([0,time_before],[0,cum[-1]],"b--",label="distance-before")
    plt.xlabel("Time /s")
    plt.ylabel("m/s or m")
    plt.legend()
    plt.show()
    p_base_list = [base_point(base_list[i,:],base_vel[i]) for i in range(len(base_list))]
    return p_base_list

spawn_position = [0,0,0]
v_max = 0.15
acceleration = 0.08
traj=pickle.load(open(sys.path[0]+'/pickle/traj2.pkl', 'rb'))

traj_rrt1,traj_rrt2,traj_rrt3,traj_aico1,traj_aico2 = traj

KDLFrame_startbase = exo.KDLFrame([spawn_position[0],spawn_position[1],0,0,0,spawn_position[2]])
base_list = np.array([my_traj_transform(traj_rrt1[i], KDLFrame_startbase) for i in range(len(traj_rrt1))])
p_base_list_1 = prepare_rrt(base_list,v_max,acceleration)

# print(p_base_list_1)