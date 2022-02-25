import numpy as np
import math
import sys
import hsrb_interface
import pyexotica as exo

from pickup_ik import pickup_ik
from pickup_rrt import pickup_rrt_loop
from pickup_aico import pickup_aico
from image.detect import detect_all
from my_functions import *
import my_arm_client as arm
import my_base_client as base

# Settings
gazebo=1
dt=0.15
vel_limit = 0.05
# Position where the robot is spawned, set in launch file "robot_pos", [x,y,Y]
spawn_position = [0,0,0]
# start_position = my_get_position(spawn_position)
start_position = [0,0,0]
end_position = spawn_position
place_position = [1,2,0.7]
scene_list_rrt = ["{hsr123}/resources/meeting_room_table.scene","{hsr123}/resources/box.scene"]
scene_list = ["{hsr123}/resources/meeting_room_table.scene","{hsr123}/resources/box.scene","{hsr123}/resources/soda_can.scene"]

# Init
if gazebo:
    try:
        robot = hsrb_interface.Robot()
        whole_body = robot.get('whole_body')
        hsrb_gripper = robot.get('gripper')
        hsrb_gripper.command(0.8)
    except:
        raise Exception("Fail to initialize")
    cli_arm, cli_base = my_Simple_Action_Clients()
    client_all = [cli_arm, cli_base, whole_body, hsrb_gripper]
    print("Looking for all the can")
    # can_position_list = detect_all(41)
# else:
    can_position_list = [[1.1883571178189685, -0.3702856919250638, 0.7998437373020126], [0.3893963418017058, -0.3677819470055569, 0.8017777247505182], [0.7914706058544925, -0.9882949445584605, 0.798380758036232]]

# A basic loop
def plan(scene_list,scene_list_rrt,start,pick,place,end,plot=1,debug=1):
    # Find the base trajectory and find the start and end for AICO pickup
    print("===Finding the base trajectory and the start and end for AICO pickup")
    base_for_pickup = pickup_ik(pick,scene_list,debug=0)
    base_for_place = pickup_ik(place,scene_list,debug=0)
    print("From pickup_ik, base_for_pickup: ",base_for_pickup[0:3])
    print("From pickup_ik: base_for_place: ",base_for_place[0:3])
    traj_rrt1_full = pickup_rrt_loop(start,base_for_pickup[0:3],scene_list_rrt,num=5,debug=0)
    traj_rrt2_full = pickup_rrt_loop(base_for_pickup[0:3],base_for_place[0:3],scene_list_rrt,num=5,debug=0)
    traj_rrt3_full = pickup_rrt_loop(base_for_place[0:3],end,scene_list_rrt,num=5,debug=0)
    traj_rrt1,traj_rrt21 = find_aico_point(traj_rrt1_full,traj_rrt2_full,0.25,base_for_pickup,0.5)
    traj_rrt2,traj_rrt3 = find_aico_point(traj_rrt21,traj_rrt3_full,0.25,base_for_place,0.5)
    aico_start1,aico_end1 = traj_rrt1[-1,:],traj_rrt21[0,:]
    aico_start2,aico_end2 = traj_rrt2[-1,:],traj_rrt3[0,:]

    # Change the orientation of the path for better grasping
    start_orientation1 = math.atan2(pick[1]-aico_start1[1],pick[0]-aico_start1[0])
    traj_rrt1[:,2] = np.linspace(traj_rrt1[0,2],start_orientation1,len(traj_rrt1),endpoint=True)
    start_orientation2 = math.atan2(place[1]-aico_start2[1],place[0]-aico_start2[0])
    traj_rrt2[:,2] = np.linspace(traj_rrt2[0,2],start_orientation2,len(traj_rrt2),endpoint=True)
    aico_start1[2]=start_orientation1
    aico_start2[2]=start_orientation2
    print("From find_aico_point, start1 and end1: ", aico_start1,aico_end1)
    print("From find_aico_point, start2 and end2: ", aico_start2,aico_end2)

    # Find the base trajectory during AICO
    print("===Finding the base trajectory during AICO")
    traj_path1=sys.path[0]+"/pickup_traj/base1.traj"
    traj_path2=sys.path[0]+"/pickup_traj/base2.traj"
    x1,y1 = my_aico_traj_new(aico_start1,aico_end1,base_for_pickup,20,traj_path1)
    x2,y2 = my_aico_traj_new(aico_start2,aico_end2,base_for_place,20,traj_path2)

    # Plot the base trajectory
    if plot:
        plt.plot(traj_rrt1_full[:,0],traj_rrt1_full[:,1],'xr')
        plt.plot(traj_rrt2_full[:,0],traj_rrt2_full[:,1],'xg')
        plt.plot(traj_rrt3_full[:,0],traj_rrt3_full[:,1],'xb')
        plt.plot(pick[0],pick[1],'og')
        plt.plot(place[0],place[1],'og')
        circle1 = plt.Circle((base_for_pickup[0],base_for_pickup[1]), 0.5, color='r',fill=False)
        circle2 = plt.Circle((base_for_place[0],base_for_place[1]), 0.5, color='r',fill=False)
        plt.gca().add_patch(circle1)
        plt.gca().add_patch(circle2)
        color1 = [str(i/len(x1)) for i in range(len(x1))]
        color2 = [str(i/len(x2)) for i in range(len(x2))]
        plt.scatter(x1,y1,s=100,c=color1)
        plt.scatter(x2,y2,s=100,c=color2)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig(sys.path[0]+"/pickup_traj/Trajectories.png")
        plt.show()
    
    print("===Finding the arm and base trajectory for grasping")
    traj_aico1 = pickup_aico(pick,traj_path1,scene_list,gripper_orientation=0,debug=0,doplot=0)
    traj_aico2 = pickup_aico(place,traj_path2,scene_list,gripper_orientation=0,debug=0,doplot=0)

    # Move in RViz
    if debug:
        arm_traj = [0,0,-np.pi/2,-np.pi/2,0]
        traj_rrt1 = np.concatenate((traj_rrt1,np.resize(arm_traj,(len(traj_rrt1),5))), axis=1)
        traj_rrt2 = np.concatenate((traj_rrt2,np.resize(arm_traj,(len(traj_rrt2),5))), axis=1)
        traj_rrt3 = np.concatenate((traj_rrt3,np.resize(arm_traj,(len(traj_rrt3),5))), axis=1)
        print("ALL DONE, Looping the solution in RViz")
        my_pickup(np.concatenate((traj_rrt1,traj_aico1,traj_rrt2,traj_aico2,traj_rrt3), axis=0),pick,scene_list)
    return [traj_rrt1,traj_rrt2,traj_rrt3,traj_aico1,traj_aico2]
    
def follow(traj,spawn_position,client,dt,vel_limit):
    traj_rrt1,traj_rrt2,traj_rrt3,traj_aico1,traj_aico2 = traj
    cli_arm, cli_base, whole_body, hsrb_gripper = client

    # Constrct base list of rrt part1
    print("Constrcting TrajectoryPoint list for rrt part1")
    KDLFrame_startbase = exo.KDLFrame([spawn_position[0],spawn_position[1],0,0,0,spawn_position[2]])
    base_list = np.array([my_traj_transform(traj_rrt1[i], KDLFrame_startbase) for i in range(len(traj_rrt1))])
    p_base_list_1 = base.prepare_rrt(base_list,vel_limit)
    # Constrct base list of aico part1
    print("Constrcting TrajectoryPoint list for aico part1")
    time_list = np.arange(0.0,dt*len(traj_aico1),dt)
    arm_list = traj_aico1[:,3:8]
    arm_list = np.c_[arm_list,time_list]
    arm_list = arm_list[1::]
    p_arm_list1 = arm.prepare_aico(arm_list,dt)
    traj_aico_base = traj_aico1[:,0:3]
    base_list = np.array([my_traj_transform(traj_aico_base[i], KDLFrame_startbase) for i in range(len(traj_aico_base))])
    base_list = np.c_[base_list, time_list]
    base_list = base_list[1::]
    p_base_list_2 = base.prepare_aico(base_list,dt)
    # Constrct base list of rrt part2
    print("Constrcting TrajectoryPoint list for  rrt part2")
    base_list = np.array([my_traj_transform(traj_rrt2[i], KDLFrame_startbase) for i in range(len(traj_rrt2))])
    p_base_list_3 = base.prepare_rrt(base_list,vel_limit)
    # Constrct base list of aico part2
    print("Constrcting TrajectoryPoint list for aico part2")
    time_list = np.arange(0.0,dt*len(traj_aico2),dt)
    arm_list = traj_aico2[:,3:8]
    arm_list = np.c_[arm_list,time_list]
    arm_list = arm_list[1::]
    p_arm_list2 = arm.prepare_aico(arm_list,dt)
    traj_aico_base = traj_aico2[:,0:3]
    base_list = np.array([my_traj_transform(traj_aico_base[i], KDLFrame_startbase) for i in range(len(traj_aico_base))])
    base_list = np.c_[base_list, time_list]
    base_list = base_list[1::]
    p_base_list_4 = base.prepare_aico(base_list,dt)
    # Constrct base list of rrt part3
    print("Constrcting TrajectoryPoint list for  rrt part3")
    base_list = np.array([my_traj_transform(traj_rrt3[i], KDLFrame_startbase) for i in range(len(traj_rrt3))])
    p_base_list_5 = base.prepare_rrt(base_list,vel_limit)
    
    # Run!!
    rospy.set_param('pickup_status', 0)
    print("Loading base goal1")
    base.load_base_goal_pickup(p_base_list_1,cli_base)
    while rospy.get_param("pickup_status")==0:
        rospy.sleep(0.1)

    print("Loading base goal2")
    base.load_base_goal_pickup(p_base_list_2,cli_base)
    arm.load_arm_goal_pickup(p_arm_list1,cli_arm)
    rospy.sleep(4*dt/0.1)
    hsrb_gripper.apply_force(0.2, delicate = True)
    while rospy.get_param("pickup_status")<3:
        rospy.sleep(0.1)

    print("Loading base goal3")
    base.load_base_goal_pickup(p_base_list_3,cli_base)
    while rospy.get_param("pickup_status")==3:
        rospy.sleep(0.1)

    print("Loading base goal4")
    base.load_base_goal_pickup(p_base_list_4,cli_base)
    arm.load_arm_goal_pickup(p_arm_list2,cli_arm)
    rospy.sleep(4*dt/0.1)
    hsrb_gripper.command(0.8)
    while rospy.get_param("pickup_status")<6:
        rospy.sleep(0.1)

    print("Loading base goal5")
    base.load_base_goal_pickup(p_base_list_5,cli_base)
    while rospy.get_param("pickup_status")==6:
        rospy.sleep(0.1)


    print('All Done :)')

traj = plan(scene_list,scene_list_rrt,start_position,can_position_list[0],place_position,end_position,plot=1,debug=1-gazebo)
follow(traj,spawn_position,client_all,dt,vel_limit)