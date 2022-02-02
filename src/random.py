#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import sys
from time import time, sleep
import signal
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import publish_pose, plot, sig_int_handler
import exotica_core_task_maps_py

from my_functions import my_transform_can, my_plot_analysis, my_get_pose
def detect_traj_aico(debug=1,tag_pose=2.5,t_grasp_begin=4.5,doplot=0):
    # Init
    exo.Setup.init_ros()
    config_name = '{hsr123}/resources/detect/aico.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    scene = problem.get_scene()
    joint_limits = problem.get_scene().get_kinematic_tree().get_joint_limits()

    # Set start states
    scene.attach_object("SodaCan", "TargetObject")
    if debug:
        scene.attach_object_local("TargetObject", "", exo.KDLFrame([0.8856, -0.3937, 0.7941]))
        scene.set_model_state_map({
            'arm_flex_joint': 0.0,
            'arm_lift_joint':0.0,
            'arm_roll_joint': -np.pi/2,
            'base_l_drive_wheel_joint':0.0,
            'base_r_drive_wheel_joint': 0.0,
            'base_roll_joint': 0.0,
            'hand_l_spring_proximal_joint': 0.9,
            'hand_motor_joint': 0.81,
            'hand_r_spring_proximal_joint': 0.9,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0,
            'wrist_flex_joint': -np.pi/2,
            'wrist_roll_joint': 0.0,})
    else:
        rospy.init_node("detect_traj_aico")
        print("Getting the location of the can from gazebo")
        scene.attach_object_local("TargetObject", "", exo.KDLFrame(my_transform_can(tag_pose)))
        print("Getting the start pose of the robot from gazebo")
        scene.set_model_state_map(my_get_pose())
    problem.start_state = scene.get_model_state()
    q_start = problem.apply_start_state(True)
    if np.any(q_start < joint_limits[:,0]) or np.any(q_start > joint_limits[:,1]):
        raise RuntimeError("Start state exceeds joint limits!")
    q_start = np.clip(q_start, joint_limits[:,0], joint_limits[:,1])
    problem.update(q_start, 0)

    # Set the Task map
    location_can = scene.fk('SodaCan').get_translation_and_rpy()
    location_gripper = scene.fk('hand_palm_link').get_translation_and_rpy()
    relative_direction = location_can-location_gripper
    gripper_orientation = [relative_direction[0],relative_direction[1]]
    gripper_orientation = gripper_orientation/(gripper_orientation[0]**2+gripper_orientation[1]**2)**0.5
    t_grasp_duration = 0.5
    T_grasp_begin = int(t_grasp_begin / problem.tau)
    T_grasp_end = int((t_grasp_begin + t_grasp_duration) / problem.tau)
    problem.get_task_maps()["EffAxisAlignment_before_grasp"].set_direction("hand_palm_link",np.append(gripper_orientation,[0.0]))
    for t in range(T_grasp_begin, T_grasp_end):
        problem.set_rho('EffPosition', 1e3, t)
        problem.set_goal('EffPosition', location_can[:3], t)
    for t in range(T_grasp_begin-10,T_grasp_begin-5):
        problem.set_rho('EffAxisAlignment_before_grasp', 1e1, t)
    for t in range(T_grasp_begin, problem.T):
        problem.set_rho('EffAxisAlignment_after_grasp', 1e3, t)
    problem.set_rho('LiftOffTable', 1e2, T_grasp_begin - 20)
    problem.set_rho('LiftOffTable', 1e2, T_grasp_end + 20)
    problem.set_rho('FinalPose', 1e3, -1)

    # Solve
    init_pose = np.zeros((problem.T,problem.N))
    for t in range(problem.T):
        init_pose[t,:] = q_start
    problem.initial_trajectory = init_pose
    solution = solver.solve()
    print("Solved in", solver.get_planning_time(), "final cost", problem.get_cost_evolution()[1][-1])
    if doplot:
        my_plot_analysis(problem,solution,scene)

    # Visualization in rviz
    if debug:
        np.save(sys.path[0]+"/trajectories/"+"detect",solution)
        midpoint = int((t_grasp_begin + t_grasp_duration)/problem.tau)
        signal.signal(signal.SIGINT, sig_int_handler)
        print("mug_location:",location_can)
        print(solution)
        t = 0
        while True:
            problem.get_scene().update(solution[t], float(t) * problem.T)
            scene.set_model_state_map({'torso_lift_joint': 0.5 * solution[t,3]})
            problem.get_scene().get_kinematic_tree().publish_frames()
            sleep(problem.tau)
            t = (t + 1) % len(solution)
            if t == midpoint:
                scene.attach_object("SodaCan", "hand_palm_link")
            elif t == 0:
                scene.attach_object_local("SodaCan", "", location_can)
    else:
        return solution


if __name__ == '__main__':
    detect_traj_aico(debug=1,tag_pose=2.5,t_grasp_begin=4.5,doplot=0)