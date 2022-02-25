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
import matplotlib.pyplot as plt
import time

def loop_rviz(problem,solution):
    '''Show solution in RViz'''
    print("Solved, looping in RViz")
    signal.signal(signal.SIGINT, sig_int_handler)
    t = 0
    while True:
        problem.get_scene().update(solution[t], float(t) * 0.1)
        # print("==========================================")
        # print(exo.tools.get_colliding_links(scene, debug=True))
        problem.get_scene().get_kinematic_tree().publish_frames()
        sleep(0.1)
        t = (t + 1) % len(solution)

def pickup_rrt(start, goal, scene_list, debug=1):
    # Init
    exo.Setup.init_ros()
    config_name = '{hsr123}/resources/pickup/rrt.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    scene = problem.get_scene()
    for i in scene_list:
        scene.load_scene_file(i)

    # Set start states
    problem.start_state = start
    problem.goal_state = goal
    scene.set_model_state(problem.start_state)

    # Solve
    solution = solver.solve()

    # Visualization in rviz
    if debug:
        np.save(sys.path[0]+"/trajectories/"+"pickup_rrt",solution)
        print(len(solution),solution)
        plt.plot(solution[:,0],solution[:,1],'xr')
        plt.show()
        loop_rviz(problem,solution)
    else:
        return solution

def pickup_rrt_loop(start, goal,scene_list,num=5,debug=1):
    '''Run RRT many times and find the best one'''
    # Init
    pickup_rrt_loop_starttime = time.time()
    exo.Setup.init_ros()
    config_name = '{hsr123}/resources/pickup/rrt.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    scene = problem.get_scene()
    for i in scene_list:
        scene.load_scene_file(i)

    # Set start states
    problem.start_state = start
    problem.goal_state = goal
    scene.set_model_state(problem.start_state)

    # Solve
    all_solution = []
    all_cost = []
    for i in range(num):
        starttime = time.time()
        solution = solver.solve()
        all_solution.append(solution)
        length = np.sum(np.sqrt(np.sum(np.diff(solution[:,0:2], axis=0)**2, axis=1)))
        all_cost.append(length)
        # Visualization in rviz
        if debug:
            plt.plot(solution[:,0],solution[:,1],'o',label=i)
            endtime = time.time()
            print(i,"---Time taken: ",round(endtime-starttime,3), ". Path length: ", round(length,3))
    solution = all_solution[min(range(len(all_cost)), key=all_cost.__getitem__)]
    print("RRT solved in: ",round(time.time()-pickup_rrt_loop_starttime,2))
    if debug:
        np.save(sys.path[0]+"/trajectories/"+"pickup_rrt",solution)
        plt.plot(solution[:,0],solution[:,1],'b--',label="Solution")
        plt.legend(loc="upper left")
        plt.show()
        loop_rviz(problem,solution)
        
    else:
        return solution


if __name__ == '__main__':
    scene_list_rrt = ["{hsr123}/resources/meeting_room_table.scene","{hsr123}/resources/box.scene"]
    # pickup_rrt([-1,-2,0],[ 0.9187 ,-0.0243, -2.0765],scene_list, debug=1)
    pickup_rrt_loop([ 1.1491, -0.0641, -1.2076],[0.9961, 1.4072, 1.4161],scene_list_rrt,num=5)