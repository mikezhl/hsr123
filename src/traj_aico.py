#!/usr/bin/env python
from __future__ import print_function, division

from time import time, sleep
import signal
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import publish_pose, plot, sig_int_handler

traj_version = 123


exo.Setup.init_ros()
config_name = '{hsr123}/resources/basic/aico.xml'
solver = exo.Setup.load_solver(config_name)
problem = solver.get_problem()
scene = problem.get_scene()
kt = scene.get_kinematic_tree()
joint_limits = problem.get_scene().get_kinematic_tree().get_joint_limits()
# Set target for soda can
scene.attach_object("SodaCan", "TargetObject")
scene.attach_object_local("TargetObject", "Table", exo.KDLFrame([0.2,0.30,0.06]))#+0.04]))

# Move robot to start state
x_start = problem.start_state
x_start[0] = 0
x_start[1] = 0
x_start[2]=  0
problem.start_state = x_start
scene.set_model_state(problem.start_state)
scene.set_model_state_map({
    'hand_motor_joint': 0.7,
    'hand_l_spring_proximal_joint':0.9,
    'hand_l_distal_joint': -0.6,
    'hand_r_spring_proximal_joint':0.9,
    'hand_r_distal_joint': -0.6,
    'wrist_roll_joint': 0,
    'wrist_flex_joint': -np.pi/2,
    'arm_roll_joint': 0})
problem.start_state = scene.get_model_state()
q_start = problem.apply_start_state(True)
q_start = np.clip(q_start, joint_limits[:,0], joint_limits[:,1])
problem.update(q_start, 0)
problem.start_state = scene.get_model_state()
q_start = problem.apply_start_state(True)
if np.any(q_start < joint_limits[:,0]) or np.any(q_start > joint_limits[:,1]):
    raise RuntimeError("Start state exceeds joint limits!")

mug_location = scene.fk('SodaCan', exo.KDLFrame(), '', exo.KDLFrame()).get_translation_and_rpy()
t_grasp_begin = 4.5
t_grasp_duration = 0.5
T_grasp_begin = int(t_grasp_begin / problem.tau)
T_grasp_end = int((t_grasp_begin + t_grasp_duration) / problem.tau)
# The target position needs to be reached during the grasping period
problem.set_rho('Position', 0, 0)
for t in range(T_grasp_begin, T_grasp_end):
    problem.set_rho('Position', 1e4, t)
    problem.set_goal('Position', mug_location[:3], t)
problem.set_rho('LiftOffTable', 1e2, T_grasp_begin - 20)
problem.set_rho('LiftOffTable', 1e2, T_grasp_end + 20)
# The axis needs to be fixed from the beginning of the grasp to the end of the motion
for t in range(T_grasp_begin, problem.T):
    problem.set_rho('AxisAlignment', 1e2, t)
problem.set_rho('BaseOrientation', 1e2, -1)

zero_motion = np.zeros((problem.T,problem.N))
for t in range(problem.T):
    zero_motion[t,:] = q_start
problem.initial_trajectory = zero_motion
solution = solver.solve()
print("Solved in", solver.get_planning_time(), "final cost", problem.get_cost_evolution()[1][-1])


midpoint = int((t_grasp_begin + t_grasp_duration)/problem.tau)

T = problem.T*problem.tau

signal.signal(signal.SIGINT, sig_int_handler)
dt = float(T) / float(len(solution))
t = 0
grasp_times = [t_grasp_begin, t_grasp_duration]
print("print(mug_location):",mug_location)
print("saving trajectory")
np.save("trajectories/"+str(traj_version),solution)
print("Looping in rviz")
while True:
    problem.get_scene().update(solution[t], float(t) * dt)
    scene.set_model_state_map({'torso_lift_joint': 0.5 * solution[t,3]})
    problem.get_scene().get_kinematic_tree().publish_frames()
    sleep(dt)
    t = (t + 1) % len(solution)
    if t == midpoint:
        scene.attach_object("SodaCan", "hand_palm_link")
    elif t == 0:
        scene.attach_object_local("SodaCan", "", mug_location)


