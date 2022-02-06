from pickup_ik import pickup_ik
from pickup_rrt import pickup_rrt
from my_functions import my_transform_can, my_plot_analysis, my_get_pose, my_rrt_loop,find_aico_point,my_bezier,my_set_traj
import rospy
import matplotlib.pyplot as plt
import numpy as np
import sys

debug=1
start=[-1,2,0]
end=[3,2,0]

if debug:
    can_position = [0.8856, -0.3937, 0.7941]
else:
    tag_pose = 2.5
    rospy.init_node("pick_up")
    print("Getting the location of the can from gazebo")
    can_position = my_transform_can(tag_pose)
base_for_pickup = pickup_ik(can_position,debug=0)
print(base_for_pickup[0][0:3])
rrt_traj1 = pickup_rrt(start,base_for_pickup[0][0:3],debug=0,doplot=0)
rrt_traj2 = pickup_rrt(base_for_pickup[0][0:3],end,debug=0,doplot=0)
rrt_traj1,rrt_traj2 = find_aico_point(rrt_traj1,rrt_traj2,0.5,can_position,1)
aico_start,aico_end = rrt_traj1[-1,:],rrt_traj2[0,:]
print(aico_start,aico_end)
traj_path="/home/hzhu/ros_123/src/hsr123/resources/pickup/base.traj"
x,y = my_bezier(can_position,[aico_start,aico_end],20,debug=0)
my_set_traj(x,y,traj_path)

plt.plot(rrt_traj1[:,0],rrt_traj1[:,1],'xr')
plt.plot(rrt_traj2[:,0],rrt_traj2[:,1],'xb')
plt.plot(can_position[0],can_position[1],'og')
plt.plot(x,y,'k')
circle1 = plt.Circle((can_position[0],can_position[1]), 1, color='r',fill=False)
plt.gca().add_patch(circle1)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

my_rrt_loop(np.concatenate((rrt_traj1,rrt_traj2), axis=0))
# np.save(sys.path[0]+"/rrt_traj1",rrt_traj1)
# np.save(sys.path[0]+"/rrt_traj2",rrt_traj2)