#!/usr/bin/env python
from __future__ import print_function, division
import sys
from time import time, sleep
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import publish_pose, plot, sig_int_handler
import exotica_core_task_maps_py
import matplotlib.pyplot as plt
from my_functions import my_ik_cost, my_random_start_ik


def pickup_ik(goal,debug=1):
    # Init
    exo.Setup.init_ros()
    config_name = '{hsr123}/resources/pickup/ik.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    scene = problem.get_scene()
    scene.load_scene_file("{hsr123}/resources/meeting_room_table.scene")
    scene.load_scene_file("{hsr123}/resources/soda_can.scene")
    
    # Set start states
    scene.attach_object("SodaCan", "TargetObject")
    scene.attach_object_local("TargetObject", "", exo.KDLFrame(goal))
    scene.set_model_state_map({
            'hand_l_spring_proximal_joint': 0.9,
            'hand_motor_joint': 0.81,
            'hand_r_spring_proximal_joint': 0.9,})
    start_state = scene.get_model_state()
        
    # Set the Task map
    location_can = scene.fk('SodaCan').get_translation_and_rpy()
    problem.set_goal('EffPosition', location_can[:3])

    # Solve with multiple start ponit
    all_solution = []
    all_cost = []
    all_cost_dict=[]
    for start in my_random_start_ik(goal[0:2],1,12):
        # Initialise in random place
        start_state[0] = start[0]
        start_state[1] = start[1]
        problem.start_state = start_state
        # Solve
        solution = solver.solve()
        cost_total,cost_dict = my_ik_cost(problem)
        all_solution.append(solution)
        all_cost.append(cost_total)
        all_cost_dict.append(cost_dict)
        # print("Total costs: ", round(cost_total,3), ". Details: ",cost_dict)

    # Find the optimal solution
    if debug:
        index = np.argmin(all_cost)
        solution = all_solution[index]
        print("List of all cost: ",all_cost," Min: ",index)
        print("Details of the cost: ", all_cost_dict[index])
    else:
        solution = all_solution[min(range(len(all_cost)), key=all_cost.__getitem__)]
    
    # Visualization in rviz
    if debug:
        # from pyexotica.tools import get_colliding_links
        # print("Colliding links", get_colliding_links(scene))
        np.save(sys.path[0]+"/trajectories/"+"pickup_ik",solution)
        print("IK solution: ", solution)
        sleep(1)
        problem.get_scene().update(solution[0], 0)
        scene.set_model_state_map({'torso_lift_joint': 0.5 * solution[0][3]})
        problem.get_scene().get_kinematic_tree().publish_frames()
        sleep(1)
    else:
        return solution[0]


if __name__ == '__main__':
    pickup_ik([0.1, -0.3937, 0.7941],debug=1)
    # pickup_ik([0.9, -1, 0.9],debug=1)