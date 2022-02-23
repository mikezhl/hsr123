import image_geometry
import rospy
from sensor_msgs.msg import CameraInfo

def get_xyz(box,debug=0):
    rospy.init_node("detect")
    CameraInfo_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/camera_info",CameraInfo)
    cam = image_geometry.PinholeCameraModel()
    cam.fromCameraInfo(CameraInfo_msg)
    middle = [int(box[0]+box[2]/2),int(box[1]+box[3]/2)]
    xyz = cam.projectPixelTo3dRay(middle)
    return xyz
if __name__ == '__main__':
    box=[541, 311,  43,  78]
    get_xyz(box,1)