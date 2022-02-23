import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def get_distance(position,debug=0):
    rospy.init_node("image")
    image_message = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/image",Image)
    middle = [int(position[1]+position[3]/2),int(position[0]+position[2]/2)]
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    distance = cv_image[middle[0],middle[1]]
    if debug:
        cv2.rectangle(cv_image, position, (0, 255, 255), 2)
        cv2.imshow("get_distance", cv_image)
    else:
        return distance

if __name__ == '__main__':
    position = [527, 319,  44,  79]
    get_distance(position,1)
    cv2.waitKey()