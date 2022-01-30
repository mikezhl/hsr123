#!/usr/bin/env python
import numpy as np
import sys
import pyexotica as exo

import rospy
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
from geometry_msgs.msg import PoseStamped
import hsrb_interface

import my_arm_client as arm
import my_base_client as base

def Simple_Action_Clients():
    '''Start and return Simple Action Clients for: arm, base.
    '''
    print("Creating Simple Action Clients")
    cli_arm = actionlib.SimpleActionClient(
        '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    cli_base = actionlib.SimpleActionClient(
        '/hsrb/omni_base_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    cli_arm.wait_for_server()
    cli_base.wait_for_server()
    
    #Make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers = (
        rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                        controller_manager_msgs.srv.ListControllers))
    running_arm = False
    running_base = False
    while (running_arm or running_base) is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'arm_trajectory_controller' and c.state == 'running':
                running_arm = True
            if c.name == 'omni_base_controller' and c.state == 'running':
                running_base = True
    print("Simple Action Clients created")
    return cli_arm, cli_base

def open_gripper(hsrb_gripper):
    # Approximate radians of open gripper
    _GRASP_OPEN = 0.8
    hsrb_gripper.command(_GRASP_OPEN)

def close_gripper(hsrb_gripper):
    # Grasp force[N]
    _GRASP_FORCE=0.2
    hsrb_gripper.apply_force(_GRASP_FORCE, delicate = True)

def transform_base_traj(base_pose, KDLFrame_base):
    '''Takes input of base_pose which is an array [x,y,rot] and a KDLFrame.
        returns [x,y,z,r,p,yaw]
    '''
    KDLFrame_traj = exo.KDLFrame(np.concatenate((base_pose[0:2],[0,0,0],[base_pose[2]])))
    KDLFrame_goal = KDLFrame_base * KDLFrame_traj
    return KDLFrame_goal.get_translation_and_rpy()

def follow(received_traj,grasp_t, dt):
    '''Trajectory from AICO: [grasp_start, grasp_duration, time step of trajectory in seconds].
    '''
    print("The shape of the trajectory:",np.shape(received_traj))
    # Initialize
    try:
        robot = hsrb_interface.Robot()
        whole_body = robot.get('whole_body')
        hsrb_gripper = robot.get('gripper')
        open_gripper(hsrb_gripper)
        whole_body.move_to_go()
    except:
        raise Exception("Fail to initialize")
    # Create Simple Action Clients for arm and base
    cli_arm, cli_base = Simple_Action_Clients()
    
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
    # Move relative to map instead of hsrb_base_link
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
    # Load the trajectory
    # soln = np.load(sys.path[0]+'/trajectories/rob.npy', allow_pickle = False)
    soln = np.load(sys.path[0]+'/trajectories/123.npy', allow_pickle = False)
    follow(soln, 4.5, 0.15)


