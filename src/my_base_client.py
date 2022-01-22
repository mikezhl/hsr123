#!/usr/bin/env python
import numpy as np
import control_msgs.msg
import rospy
import trajectory_msgs.msg

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

def base_point(current_base_traj, current_base_velocity):
    p_base = trajectory_msgs.msg.JointTrajectoryPoint()
    p_base.velocities = current_base_velocity[0:3]
    p_base.positions = current_base_traj[0:3]
    p_base.time_from_start = rospy.Time(current_base_traj[3])
    return p_base

def calculate_velocity(pos_array, dt):
    pos_array = np.append(pos_array, [pos_array[-1]], axis=0)
    vel_array = np.diff(pos_array, axis = 0)/dt
    return vel_array

def load_base_goal(base_list,cli_base,dt):
    # base_list, list of base trajectories with form [base angle x3, time from start]
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