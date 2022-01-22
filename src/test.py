import rospy
import hsrb_interface
from geometry_msgs.msg import PoseStamped
import pyexotica as exo
import numpy as np
import sys
import pickle
def transform_base_traj(base_pose, tf_base):
    '''takes input of base_pose which is an array [x,y,rot] and a KDLFrame.
        returns [x,y,z,r,p,yaw]
    '''
    frame = exo.KDLFrame(np.concatenate((base_pose[0:2],[0,0,0],[base_pose[2]])))
    rel_goal = tf_base * frame
    return rel_goal.get_translation_and_rpy()

received_traj = np.load(sys.path[0]+'/trajectory/rob.npy', allow_pickle = False)
xyy_traj = received_traj[10,0:3]
KDLF_tra = exo.KDLFrame(np.concatenate((xyy_traj[0:2],[0,0,0],[xyy_traj[2]])))

base_pose = pickle.load( open( "save.p", "rb" ) )
x = base_pose.position.x
y = base_pose.position.y
z = base_pose.position.z
qx= base_pose.orientation.x
qy= base_pose.orientation.y
qz= base_pose.orientation.z
qw= base_pose.orientation.w
KDLF_base = exo.KDLFrame([x,y,z,qx,qy,qz,qw])

KDLF_result = KDLF_base * KDLF_tra
re = KDLF_result.get_translation_and_rpy()


print(re)