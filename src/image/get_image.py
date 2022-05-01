import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def get_image(debug=0):
    image_message = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    if debug==1:
        cv2.imshow("Image window", cv_image)
        cv2.waitKey()
    else:
        return cv_image

if __name__ == '__main__':
    rospy.init_node("test")
    get_image(1)