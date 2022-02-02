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
def detect_traj_aico(debug=1,doplot=0):
    # Init
    exo.Setup.init_ros()
    config_name = '{hsr123}/resources/rrt/rrt.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    scene = problem.get_scene()

    # Set start states
    problem.start_state = [0,0,0]
    scene.set_model_state(problem.start_state)

    # Solve
    solution = solver.solve()
    if doplot:
        my_plot_analysis(problem,solution,scene)

    # Visualization in rviz
    if debug:
        np.save(sys.path[0]+"/trajectories/"+"rrt",solution)
        signal.signal(signal.SIGINT, sig_int_handler)
        print(solution)
        t = 0
        while True:
            problem.get_scene().update(solution[t], float(t) * 0.1)
            problem.get_scene().get_kinematic_tree().publish_frames()
            sleep(0.1)
            t = (t + 1) % len(solution)
    else:
        return solution


if __name__ == '__main__':
    detect_traj_aico(debug=1,doplot=0)