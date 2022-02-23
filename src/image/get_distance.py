import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def get_distance(box,debug=0):
    rospy.init_node("detect")
    image_message = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/image",Image)
    middle = [int(box[1]+box[3]/2),int(box[0]+box[2]/2)]
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    distance = cv_image[middle[0],middle[1]]
    if debug:
        cv2.rectangle(cv_image, box, (0, 255, 255), 2)
        cv2.imshow("get_distance", cv_image)
    return distance

if __name__ == '__main__':
    box = [527, 319,  44,  79]
    get_distance(box,1)
    cv2.waitKey()