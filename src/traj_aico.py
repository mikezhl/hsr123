#!/usr/bin/env python
from __future__ import print_function, division
import sys
from time import time, sleep
import signal
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import publish_pose, plot, sig_int_handler

traj_version = 123

# Init
exo.Setup.init_ros()
config_name = '{hsr123}/resources/basic/aico.xml'
solver = exo.Setup.load_solver(config_name)
problem = solver.get_problem()
scene = problem.get_scene()
joint_limits = problem.get_scene().get_kinematic_tree().get_joint_limits()

# Set start states
scene.attach_object("SodaCan", "TargetObject")
scene.attach_object_local("TargetObject", "Table", exo.KDLFrame([0.2,0.30,0.06]))#+0.04]))
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
problem.start_state = scene.get_model_state()
q_start = problem.apply_start_state(True)
if np.any(q_start < joint_limits[:,0]) or np.any(q_start > joint_limits[:,1]):
    raise RuntimeError("Start state exceeds joint limits!")
q_start = np.clip(q_start, joint_limits[:,0], joint_limits[:,1])
problem.update(q_start, 0)

# Set the Task map
mug_location = scene.fk('SodaCan').get_translation_and_rpy()
t_grasp_begin = 4.5
t_grasp_duration = 0.5
T_grasp_begin = int(t_grasp_begin / problem.tau)
T_grasp_end = int((t_grasp_begin + t_grasp_duration) / problem.tau)
for t in range(T_grasp_begin, T_grasp_end):
    problem.set_rho('EffPosition', 1e3, t)
    problem.set_goal('EffPosition', mug_location[:3], t)
for t in range(T_grasp_begin-10, problem.T):
    problem.set_rho('EffAxisAlignment', 1e3, t)
problem.set_rho('LiftOffTable', 1e2, T_grasp_begin - 20)
problem.set_rho('LiftOffTable', 1e2, T_grasp_end + 20)
problem.set_rho('FinalPose', 1e3, -1)

init_pose = np.zeros((problem.T,problem.N))
for t in range(problem.T):
    init_pose[t,:] = q_start
problem.initial_trajectory = init_pose

solution = solver.solve()
print("Solved in", solver.get_planning_time(), "final cost", problem.get_cost_evolution()[1][-1])


midpoint = int((t_grasp_begin + t_grasp_duration)/problem.tau)
signal.signal(signal.SIGINT, sig_int_handler)
t = 0
print("mug_location:",mug_location)
np.save(sys.path[0]+"/trajectories/"+str(traj_version),solution)
print("Trajectory saved")
while True:
    problem.get_scene().update(solution[t], float(t) * problem.T)
    scene.set_model_state_map({'torso_lift_joint': 0.5 * solution[t,3]})
    problem.get_scene().get_kinematic_tree().publish_frames()
    sleep(problem.tau)
    t = (t + 1) % len(solution)
    if t == midpoint:
        scene.attach_object("SodaCan", "hand_palm_link")
    elif t == 0:
        scene.attach_object_local("SodaCan", "", mug_location)


