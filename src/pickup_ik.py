#!/usr/bin/env python
from __future__ import print_function, division
import sys
from time import time, sleep
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import publish_pose, plot, sig_int_handler
import exotica_core_task_maps_py
import matplotlib.pyplot as plt
from my_functions import my_ik_cost
def pickup_ik(can_position,debug=1):
    # Init
    exo.Setup.init_ros()
    config_name = '{hsr123}/resources/pickup/ik.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    scene = problem.get_scene()

    # Set start states
    scene.attach_object("SodaCan", "TargetObject")
    scene.attach_object_local("TargetObject", "", exo.KDLFrame(can_position))
    scene.set_model_state_map({
            # 'world_joint/x': 0.8856,
            # 'world_joint/y': -0.3937,
            # 'world_joint/theta': 0,
            'hand_l_spring_proximal_joint': 0.9,
            'hand_motor_joint': 0.81,
            'hand_r_spring_proximal_joint': 0.9,})
    problem.start_state = scene.get_model_state()
    
    # Set the Task map
    location_can = scene.fk('SodaCan').get_translation_and_rpy()
    problem.set_goal('EffPosition', location_can[:3])

    # Solve
    solution = solver.solve()
    print(my_ik_cost(problem))



    # Visualization in rviz
    if debug:
        np.save(sys.path[0]+"/trajectories/"+"ik",solution)
        print(solution)
        sleep(1)
        problem.get_scene().update(solution[0], 0)
        scene.set_model_state_map({'torso_lift_joint': 0.5 * solution[0][3]})
        problem.get_scene().get_kinematic_tree().publish_frames()
        sleep(1)
    else:
        return solution


if __name__ == '__main__':
    pickup_ik([0.8856, -0.3937, 0.7941],debug=1)