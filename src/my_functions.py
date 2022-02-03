import rospy
import tf2_ros
import numpy as np
import signal
from pyexotica.publish_trajectory import sig_int_handler
import math

def my_transform_can(rotation):
    import math
    '''Return the position of the can relative to map, "rotation" is the Y set the launch'''
    x_offset = math.sin(rotation-math.pi/2)*0.033
    y_offset = math.cos(rotation-math.pi/2)*0.033
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    x=0; y=0; z=0; n=0
    signal.signal(signal.SIGINT, sig_int_handler)
    while True:
        try:
            can_abs = tfBuffer.lookup_transform("map","tag_0", rospy.Time())
            if n<11:
                x += can_abs.transform.translation.x
                y += can_abs.transform.translation.y
                z += can_abs.transform.translation.z
                n += 1
            else:
                break
        except:
            continue
    return [round(num, 4) for num in [x/n+x_offset, y/n-y_offset, z/n]]

def my_plot_analysis(problem,solution,scene):
    import matplotlib.pyplot as plt
    '''Plot some fig about the solver for analysis'''
    # Show convergence plot
    fig = plt.figure(1)
    plt.plot(problem.get_cost_evolution()[0], problem.get_cost_evolution()[1])
    plt.yscale('log')
    plt.ylabel('Cost')
    plt.xlabel('Time (s)')
    plt.xlim(0,np.max(problem.get_cost_evolution()[0]))
    plt.title('Convergence')

    # Show cost breakdown
    fig = plt.figure(2)

    costs = {}
    ct = 1.0 / problem.tau / problem.T
    for t in range(problem.T):
        problem.update(solution[t,:],t)
    for cost_task in problem.cost.indexing:
        task = problem.cost.tasks[cost_task.id]
        task_name = task.name
        task_id = task.id
        costs[task_name] = np.zeros((problem.T,))
        # print(task_id, task_name, task, cost_task.start, cost_task.length, cost_task.startJ, cost_task.lengthJ)
        for t in range(problem.T):
            ydiff = problem.cost.ydiff[t][cost_task.startJ:cost_task.startJ+cost_task.lengthJ]
            rho = problem.cost.S[t][cost_task.startJ:cost_task.startJ+cost_task.lengthJ,cost_task.startJ:cost_task.startJ+cost_task.lengthJ]
            cost = np.dot(np.dot(ydiff, rho), ydiff)
            costs[task_name][t] = ct * cost

    costs['Task'] = np.zeros((problem.T,))
    costs['Transition'] = np.zeros((problem.T,))
    for t in range(problem.T):
        costs['Task'][t] = problem.get_scalar_task_cost(t)
        costs['Transition'][t] = problem.get_scalar_transition_cost(t)
    for cost in costs:
        plt.plot(costs[cost], label=cost)
    plt.legend()
    plt.xlim(0,problem.T)
    plt.title('Cost breakdown across trajectory per task')
    plt.show()

def my_get_pose():
    from sensor_msgs.msg import JointState
    '''Get the pose of the robot'''
    sub = rospy.wait_for_message("/hsrb/joint_states",JointState)
    name = sub.name
    position = sub.position
    state={}
    for i in range(len(name)):
        state[name[i]] = position[i]
    return state

def my_Simple_Action_Clients():
    '''Start and return Simple Action Clients for: arm, base.
    '''
    import actionlib
    import control_msgs.msg
    import controller_manager_msgs.srv
    print("Creating Simple Action Clients")
    cli_arm = actionlib.SimpleActionClient(
        '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    cli_base = actionlib.SimpleActionClient(
        '/hsrb/omni_base_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    cli_arm.wait_for_server()
    cli_base.wait_for_server()
    
    #Make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers = (
        rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                        controller_manager_msgs.srv.ListControllers))
    running_arm = False
    running_base = False
    while (running_arm or running_base) is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'arm_trajectory_controller' and c.state == 'running':
                running_arm = True
            if c.name == 'omni_base_controller' and c.state == 'running':
                running_base = True
    print("Simple Action Clients created")
    return cli_arm, cli_base

def my_rrt_loop(solution):
    from time import sleep
    import pyexotica as exo
    exo.Setup.init_ros()
    config_name = '{hsr123}/resources/rrt.xml'
    solver = exo.Setup.load_solver(config_name)
    problem = solver.get_problem()
    signal.signal(signal.SIGINT, sig_int_handler)
    t = 0
    while True:
        problem.get_scene().update(solution[t], float(t) * 0.1)
        # print("==========================================")
        # print(exo.tools.get_colliding_links(scene, debug=True))
        problem.get_scene().get_kinematic_tree().publish_frames()
        sleep(0.1)
        t = (t + 1) % len(solution)

def find_turn(traj,max_change):
    vec = np.diff(traj, axis=0)
    deg=np.zeros(len(vec))
    for i in range(len(vec)):
        deg[i] = math.atan2(vec[i,0],vec[i,1])
    change = list(np.diff(deg))
    change.reverse()
    n=0
    for i in change:
        if i>max_change:
            return n
        else:
            n += 1
    return n
def find_close(traj,point,max_distance):
    mylist = np.flip(traj,axis=0)
    n=0
    for i in mylist:
        dis = ((i[0]-point[0])**2+(i[1]-point[1])**2)**0.5
        if dis>max_distance:
            return n
        else:
            n +=1
def find_aico_point(traj1,traj2,max_change,point,max_distance):
    one = min(find_turn(traj1,max_change),find_close(traj1,point,max_distance))
    two = min(find_turn(np.flip(traj2,axis=0),max_change),find_close(np.flip(traj2,axis=0),point,max_distance))
    return traj1[-one,:],traj2[two,:]




