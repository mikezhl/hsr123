import numpy as np
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from image.detect import detect,get_vector,broadcast_once
import hsrb_interface

def phy_detect():
    robot = hsrb_interface.Robot()
    whole_body = robot.get('whole_body')
    print("Start detecting around")
    target_list=[]
    target_id=41

    for i in [0]:
        print("Detecting angle: ", i)
        whole_body.move_to_joint_positions({"head_tilt_joint":0,'head_pan_joint': i})
        rospy.sleep(5)
        temp_list = detect(target_id)
        # print("For angle ",i,"there are: ",temp_list)
        current_pose = whole_body.joint_state.position[9:11]
        temp_target=temp_list[0]
        print(temp_target)
        target_vector = get_vector(temp_target)
        yaw = np.arctan2(target_vector[0],target_vector[2])
        pitch = np.arctan2(-target_vector[1],np.sqrt(target_vector[0]**2+target_vector[2]**2))
        # print(float(current_pose[0])-yaw,float(current_pose[1])+pitch)
        whole_body.move_to_joint_positions({'head_pan_joint': float(current_pose[0])-yaw,"head_tilt_joint":float(current_pose[1])+pitch})
        rospy.sleep(5)
        target_vector_new=get_vector([[],[319, 239, 2, 2]])
        target_point = PointStamped()
        target_point.header.frame_id = "head_rgbd_sensor_link"
        target_point.point.x = target_vector_new[0]
        target_point.point.y = target_vector_new[1]
        target_point.point.z = target_vector_new[2]
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        rospy.sleep(3)
        target_point.header.stamp = rospy.Time.now()
        rospy.sleep(3)
        point_target = buffer.transform(target_point,"odom")
        xyz = [point_target.point.x,point_target.point.y-0.02,point_target.point.z+0.05]
        target_list.append(xyz)
        print("New: ",xyz)
        broadcast_once(xyz,"odom","target-"+str(len(target_list)))
    whole_body.move_to_joint_positions({"head_tilt_joint":0,'head_pan_joint': 0})
    return target_list

if __name__ == '__main__':
    rospy.init_node("phy_detect")
    print(phy_detect())