#!/usr/bin/env python
import numpy as np
import control_msgs.msg
import rospy
import trajectory_msgs.msg

# Three callback functions for the arm action client
def active_cb_arm():
    return
    print('[ACTIVE_ARM] arm goal active')
def feedback_cb_arm(feedback):
    return
    print('[FEEDBACK] :' + str(feedback))
def done_cb_arm(state, result):
    print('[DONE_ARM] the state is: '+str(state))
    print('[DONE_ARM] the result is: '+str(result))
    if state == 3:
        print('[ARM FINISHED]')
        if rospy.get_param("status_check")==0:
            rospy.set_param('status_check', 1)
        else:
            rospy.signal_shutdown('Finished')
        return
    else:
        print('Issue arose, shutting down')
# Generate TrajectoryPoint for arm action client
def arm_point(current_arm_traj, current_arm_velocity):
    p_arm = trajectory_msgs.msg.JointTrajectoryPoint()
    p_arm.velocities = current_arm_velocity[0:5]
    p_arm.positions = current_arm_traj[0:5]
    p_arm.time_from_start = rospy.Time(current_arm_traj[5])
    return p_arm
# Calculate velocity for arm action client
def calculate_velocity(pos_array, dt):
    pos_array = np.append(pos_array, [pos_array[-1]], axis=0)
    vel_array = np.diff(pos_array, axis = 0)/dt
    return vel_array
# Constuct the TrajectoryPoint and send to the arm action client
def load_arm_goal(arm_list,cli_arm,dt):
    '''arm_list in the form of [joint angle x5, time from start]'''
    rospy.set_param('status_check', 0)
    if arm_list.any():
        arm_vel = calculate_velocity(arm_list,dt)
        arm_pos = arm_list
        goal_arm = control_msgs.msg.FollowJointTrajectoryGoal()
        traj_arm = trajectory_msgs.msg.JointTrajectory()
        traj_arm.joint_names = ["arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p_arm_list = [arm_point(arm_pos[i,:], arm_vel[i,:]) for i in range(len(arm_list))]
        traj_arm.points = p_arm_list
        goal_arm.trajectory = traj_arm
        cli_arm.send_goal(goal_arm,done_cb=done_cb_arm, active_cb=active_cb_arm, feedback_cb=feedback_cb_arm)
        return


# Callback functions for the arm action client used in pickup
def done_cb_arm_pickup(state, result):
    # print('[DONE_ARM] the state is: '+str(state))
    # print('[DONE_ARM] the result is: '+str(result))
    if state == 3:
        # print('[ARM FINISHED]')
        from std_srvs.srv import Empty
        client = rospy.ServiceProxy('Increment', Empty)
        client()
        return
    else:
        print('Issue arose, shutting down')
# Generate TrajectoryPoint list for arm action client used in pickup
def prepare_aico(arm_list,dt):
    '''arm_list in the form of [joint angle x5, time from start]'''
    arm_vel = calculate_velocity(arm_list,dt)
    p_arm_list = [arm_point(arm_list[i,:], arm_vel[i,:]) for i in range(len(arm_list))]
    return p_arm_list
# Send the TrajectoryPoint list to the arm action client used in pickup
def load_arm_goal_pickup(p_arm_list,cli_arm):
    goal_arm = control_msgs.msg.FollowJointTrajectoryGoal()
    traj_arm = trajectory_msgs.msg.JointTrajectory()
    traj_arm.joint_names = ["arm_lift_joint", "arm_flex_joint",
                "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    traj_arm.points = p_arm_list
    goal_arm.trajectory = traj_arm
    cli_arm.send_goal(goal_arm,done_cb=done_cb_arm_pickup, active_cb=active_cb_arm, feedback_cb=feedback_cb_arm)
    return