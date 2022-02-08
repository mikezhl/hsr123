#!/usr/bin/env python
import numpy as np
import control_msgs.msg
import rospy
import trajectory_msgs.msg

# Three callback functions for the arm action client
def active_cb_base():
    print('[ACTIVE_BASE] goal active')
def feedback_cb_base(feedback):
    return
    print('[FEEDBACK] :' + str(feedback))
def done_cb_base(state, result):
    print('[DONE_BASE] the state is: '+str(state))
    print('[DONE_BASE] the result is: '+str(result))
    if state == 3:
        print('[BASE FINISHED]')
        if rospy.get_param("status_check")==0:
            rospy.set_param('status_check', 1)
        else:
            rospy.signal_shutdown('Finished')
        return
    else:
        print('Issue arose, shutting down')
# Generate TrajectoryPoint for base action client
def base_point(current_base_traj, current_base_velocity):
    p_base = trajectory_msgs.msg.JointTrajectoryPoint()
    p_base.velocities = current_base_velocity[0:3]
    p_base.positions = current_base_traj[0:3]
    p_base.time_from_start = rospy.Time(current_base_traj[3])
    return p_base
# Calculate velocity for base action client
def calculate_velocity(pos_array, dt):
    pos_array = np.append(pos_array, [pos_array[-1]], axis=0)
    vel_array = np.diff(pos_array, axis = 0)/dt
    return vel_array
# Constuct the TrajectoryPoint and send to the base action client
def load_base_goal(base_list,cli_base,dt):
    '''base_list in the form of [base angle x3, time from start]'''
    if base_list.any():
        rospy.set_param('status_check', 0)
        base_vel = calculate_velocity(base_list, dt)
        base_pos = base_list
        goal_base = control_msgs.msg.FollowJointTrajectoryGoal()
        traj_base = trajectory_msgs.msg.JointTrajectory()
        traj_base.header.frame_id = "base_link"
        traj_base.joint_names = ["odom_x", "odom_y", "odom_t"]
        p_base_list = [base_point(base_pos[i,:],base_vel[i,:]) for i in range(len(base_list))]
        traj_base.points = p_base_list
        goal_base.trajectory = traj_base
        cli_base.send_goal(goal_base,done_cb=done_cb_base, active_cb=active_cb_base, feedback_cb=feedback_cb_base)
        return


# Callback functions for the base action client used in pickup
def done_cb_base_pickup(state, result):
    # print('[DONE_BASE] the state is: '+str(state))
    # print('[DONE_BASE] the result is: '+str(result))
    if state == 3:
        print('[BASE FINISHED]')
        n = int(rospy.get_param("pickup_status"))+1
        rospy.set_param('pickup_status', n)
        return
    else:
        print('Issue arose, shutting down')
# Generate TrajectoryPoint list from aico planning result for arm action client used in pickup
def prepare_aico(base_list,dt):
    '''base_list in the form of [base angle x3, time from start]'''
    base_vel = calculate_velocity(base_list, dt)
    p_base_list = [base_point(base_list[i,:],base_vel[i,:]) for i in range(len(base_list))]
    return p_base_list
# Generate TrajectoryPoint list from rrt planning result for arm action client used in pickup
def prepare_rrt(base_list,vel_limit):
    '''base_list in the form of [base angle x3]'''
    distance = np.diff(base_list, axis = 0)
    distance_abs = (distance[:,0]**2+distance[:,1]**2)**0.5
    dt_list = distance_abs/vel_limit
    time_list = np.cumsum(dt_list)
    base_list = np.c_[base_list, np.concatenate(([0],time_list))]
    base_vel  = [(base_list[i+1,0:3]-base_list[i,0:3])/(base_list[i+1,3]-base_list[i,3]) for i in range(len(base_list)-1)]
    base_list = base_list[1::]
    p_base_list = [base_point(base_list[i,:],base_vel[i]) for i in range(len(base_list))]
    return p_base_list
# Send the TrajectoryPoint list to the base action client used in pickup
def load_base_goal_pickup(p_base_list,cli_base):
    goal_base = control_msgs.msg.FollowJointTrajectoryGoal()
    traj_base = trajectory_msgs.msg.JointTrajectory()
    traj_base.header.frame_id = "base_link"
    traj_base.joint_names = ["odom_x", "odom_y", "odom_t"]
    traj_base.points = p_base_list
    goal_base.trajectory = traj_base
    cli_base.send_goal(goal_base,done_cb=done_cb_base_pickup, active_cb=active_cb_base, feedback_cb=feedback_cb_base)
    return