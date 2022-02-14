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

from my_functions import my_transform_can, my_plot_analysis, my_bezier,my_set_traj
def pickup_aico(can_position,traj_path,gripper_orientation,debug=1,doplot=0):
    # Init
    exo.Setup.init_ros()
    t_grasp_begin=4
    config_name = '{hsr123}/resources/pickup/aico.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    scene = problem.get_scene()
    scene.add_trajectory_from_file("BaseTarget",traj_path)
    BaseTarget_split = scene.get_trajectory("BaseTarget").split("\n")
    start = BaseTarget_split[2].split()
    end = BaseTarget_split[-1].split()
    joint_limits = problem.get_scene().get_kinematic_tree().get_joint_limits()

    # Set start states
    scene.attach_object("SodaCan", "TargetObject")
    scene.attach_object_local("TargetObject", "", exo.KDLFrame(can_position))
    scene.set_model_state_map({
        'world_joint/x': float(start[1]),
        'world_joint/y': float(start[2]),
        'world_joint/theta': float(start[3]),
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
    problem.start_state = scene.get_model_state()
    q_start = problem.apply_start_state(True)
    if np.any(q_start < joint_limits[:,0]) or np.any(q_start > joint_limits[:,1]):
        raise RuntimeError("Start state exceeds joint limits!")
    q_start = np.clip(q_start, joint_limits[:,0], joint_limits[:,1])
    problem.update(q_start, 0)

    # Set the Task map
    location_can = scene.fk('SodaCan').get_translation_and_rpy()
    if isinstance(gripper_orientation,int):
        location_gripper = scene.fk('hand_palm_link').get_translation_and_rpy()
        relative_direction = location_can-location_gripper
        gripper_orientation = [relative_direction[0],relative_direction[1]]
        gripper_orientation = gripper_orientation/(gripper_orientation[0]**2+gripper_orientation[1]**2)**0.5
    t_grasp_duration = 1.5
    T_grasp_begin = int(t_grasp_begin / problem.tau)
    T_grasp_end = int((t_grasp_begin + t_grasp_duration) / problem.tau)
    problem.get_task_maps()["EffAxisAlignment_before_grasp"].set_direction("hand_palm_link",np.append(gripper_orientation,[0.0]))
    problem.get_task_maps()["FinalPose"].joint_ref = np.array([float(end[1]),float(end[2]),float(end[3]),  0.  ,  0.  , -1.57, -1.57,  0.  ])
    for t in range(T_grasp_begin, T_grasp_end):
        problem.set_rho('EffPosition', 1e3, t)
        problem.set_goal('EffPosition', location_can[:3], t)
    for t in range(T_grasp_begin-15,T_grasp_begin-10):
        problem.set_rho('EffAxisAlignment_before_grasp', 100, t)
    for t in range(T_grasp_begin, problem.T):
        problem.set_rho('EffAxisAlignment_after_grasp', 1e3, t)
    for t in range(10, problem.T-10):
        problem.set_rho('StayHeight', 1e2, t)
    problem.set_rho('FinalPose', 1e3, -1)

    # Solve
    init_pose = np.zeros((problem.T,problem.N))
    for t in range(problem.T):
        init_pose[t,:] = q_start
    problem.initial_trajectory = init_pose
    solution = solver.solve()
    if doplot:
        my_plot_analysis(problem,solution,scene)

    # Visualization in rviz
    if debug:
        print("Solved in", solver.get_planning_time(), "final cost", problem.get_cost_evolution()[1][-1])
        np.save(sys.path[0]+"/trajectories/"+"pickup_aico",solution)
        midpoint = int((t_grasp_begin + t_grasp_duration)/problem.tau)
        signal.signal(signal.SIGINT, sig_int_handler)
        print("mug_location:",location_can)
        # print(solution)
        print(gripper_orientation)
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
    traj_path=sys.path[0]+"/pickup_traj/base1.traj"
    pickup_aico([0.1, -0.3937, 0.7941],traj_path,0,debug=1,doplot=1)