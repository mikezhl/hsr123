import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def get_distance(box,debug=0):
    image_message = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/image",Image)
    middle = [int(box[0]+box[2]/2),int(box[1]+box[3]/2)]
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    distance = cv_image[middle[1],middle[0]]
    if debug:
        cv2.rectangle(cv_image, box, (0, 255, 255), 2)
        cv2.imshow("get_distance", cv_image)
    return distance+0.02 # Radius of the can

if __name__ == '__main__':
    box = [541, 311,  43,  78]
    get_distance(box,1)
    cv2.waitKey()