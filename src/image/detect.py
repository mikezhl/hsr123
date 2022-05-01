import numpy as np
import cv2
import signal
from pyexotica.publish_trajectory import sig_int_handler
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped

from get_image import get_image
from get_distance import get_distance
from get_xyz import get_xyz

# Do init_node before using detect

def format_yolov5(frame):
    '''Return the image in the format required by yolov5'''
    row, col, _ = frame.shape
    _max = max(col, row)
    result = np.zeros((_max, _max, 3), np.uint8)
    result[0:row, 0:col] = frame
    return result
def detect(target_id,debug=0):
    '''Detect the object with the target_id in classes.txt'''
    c=0.01
    # Load the model and feed a 640x640 image to get predictions
    import os
    dirname = os.path.dirname(__file__)
    net = cv2.dnn.readNet(os.path.join(dirname, 'yolo_models/yolov5s.onnx'))
    image = get_image()
    input_image = format_yolov5(image) # making the image square
    blob = cv2.dnn.blobFromImage(input_image , 1/255.0, (640, 640), swapRB=True)
    net.setInput(blob)
    predictions = net.forward()
    # Unwrap the predictions to get the object detections 
    class_ids = []
    confidences = []
    boxes = []
    output_data = predictions[0]
    image_width, image_height, _ = input_image.shape
    x_factor = image_width / 640
    y_factor =  image_height / 640
    for r in range(25200):
        row = output_data[r]
        confidence = row[4]
        if confidence >= c:
            # print(confidence)
            classes_scores = row[5:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > .25):
                # print("classes_scores: ",classes_scores[class_id],"ID: ",class_id)
                confidences.append(confidence)
                class_ids.append(class_id)
                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                box = np.array([left, top, width, height])
                boxes.append(box)
    class_list = []
    with open(os.path.join(dirname, 'yolo_models/classes.txt'), "r") as f:
        class_list = [cname.strip() for cname in f.readlines()]
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0, 0)
    result_class_ids = []
    result_confidences = []
    result_boxes = []
    target_list = []
    for i in indexes:
        # print("Class: ",class_ids[i]," Confidence: ",confidences[i]," Position: ",boxes[i])
        result_confidences.append(confidences[i])
        result_class_ids.append(class_ids[i])
        result_boxes.append(boxes[i])
        if class_ids[i] == target_id:
            target_list.append([confidences[i],boxes[i]])
    # if len(target_list)==0:
    #     print("Target not found")
    # else:
    #     print("Target reuslt list: ",target_list)
    # Plot the result
    if debug:
        for i in range(len(result_class_ids)):
            box = result_boxes[i]
            class_id = result_class_ids[i]
            cv2.rectangle(image, box, (0, 255, 255), 2)
            cv2.rectangle(image, (box[0], box[1] - 20), (box[0] + box[2], box[1]), (0, 255, 255), -1)
            cv2.putText(image, class_list[class_id], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
        cv2.imshow("output", image)
        cv2.waitKey()
    return target_list
def get_vector(target):
    '''Return the vector from the camera to the object'''
    target_box = target[1]
    target_distance = get_distance(target_box)
    target_direction = np.array(get_xyz(target_box))
    return target_direction*target_distance
def detect_around_calibrated(target_id):
    '''Look around, find the object, return the postion in [x,y,z]'''
    import hsrb_interface
    robot = hsrb_interface.Robot()
    whole_body = robot.get('whole_body')
    print("Start detecting around")
    for i in [0,np.pi/4,np.pi/2,-np.pi/4,-np.pi/2,-np.pi/4*3,-np.pi,-3.839]:
        print("Detecting angle: ", i)
        whole_body.move_to_joint_positions({"head_tilt_joint":0,'head_pan_joint': i})
        rospy.sleep(3)
        target_list = detect(target_id)
        if len(target_list)!=0:
            print("Found")
            break
    target_vector = get_vector(target_list[0])
    yaw = np.arctan2(target_vector[0],target_vector[2])
    pitch = np.arctan2(-target_vector[1],np.sqrt(target_vector[0]**2+target_vector[2]**2))
    robot = hsrb_interface.Robot()
    whole_body = robot.get('whole_body')
    current_pose = whole_body.joint_state.position[9:11]
    whole_body.move_to_joint_positions({'head_pan_joint': float(current_pose[0])-yaw,"head_tilt_joint":float(current_pose[1])+pitch})

    target_vector = get_vector(detect(41)[0])
    target_point = PointStamped()
    target_point.header.frame_id = "head_rgbd_sensor_link"
    target_point.point.x = target_vector[0]
    target_point.point.y = target_vector[1]
    target_point.point.z = target_vector[2]
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(0.2)
    target_point.header.stamp = rospy.Time.now()
    rospy.sleep(0.2)
    point_target = buffer.transform(target_point,"map")
    return [point_target.point.x,point_target.point.y,point_target.point.z]
def detect_all(target_id):
    '''Look around, find the all the object in the scene, return the postion in [[x,y,z],...]'''
    import hsrb_interface
    robot = hsrb_interface.Robot()
    whole_body = robot.get('whole_body')
    print("Start detecting around")
    target_list=[]
    for i in [0,np.pi/4,np.pi/2,-np.pi/4,-np.pi/2,-np.pi/4*3,-np.pi,-3.839]:
        print("Detecting angle: ", i)
        whole_body.move_to_joint_positions({"head_tilt_joint":0,'head_pan_joint': i})
        rospy.sleep(5)
        temp_list = detect(target_id)
        # print("For angle ",i,"there are: ",temp_list)
        current_pose = whole_body.joint_state.position[9:11]
        for temp_target in temp_list:
            target_vector = get_vector(temp_target)
            yaw = np.arctan2(target_vector[0],target_vector[2])
            pitch = np.arctan2(-target_vector[1],np.sqrt(target_vector[0]**2+target_vector[2]**2))
            robot = hsrb_interface.Robot()
            whole_body = robot.get('whole_body')
            # print(float(current_pose[0])-yaw,float(current_pose[1])+pitch)
            whole_body.move_to_joint_positions({'head_pan_joint': float(current_pose[0])-yaw,"head_tilt_joint":float(current_pose[1])+pitch})
            rospy.sleep(4)
            target_vector_new=get_vector([[],[319, 239, 2, 2]])
            # status = 0
            # temp_temp_target_list = detect(41)
            # # print("temp_temp_target_list",temp_temp_target_list)
            # for temp_temp_target in temp_temp_target_list:
            #     temp_target_vector = get_vector(temp_temp_target)
            #     # print(temp_target_vector)
            #     if abs(temp_target_vector[0])<0.04 and abs(temp_target_vector[1])<0.04:
            #         target_vector_new = temp_target_vector
            #         status = 1
            # if status==0:
            #     print("Failed to find distance")
            #     continue
            target_point = PointStamped()
            target_point.header.frame_id = "head_rgbd_sensor_link"
            target_point.point.x = target_vector_new[0]
            target_point.point.y = target_vector_new[1]
            target_point.point.z = target_vector_new[2]
            buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buffer)
            rospy.sleep(0.2)
            target_point.header.stamp = rospy.Time.now()
            rospy.sleep(0.2)
            point_target = buffer.transform(target_point,"map")
            xyz = [point_target.point.x,point_target.point.y,point_target.point.z]
            new=1
            for exist in target_list:
                dis = np.sum((np.array(xyz)-np.array(exist))**2)**0.5
                if dis<0.1:
                    new=0
                    break
            if new:
                target_list.append(xyz)
                print("New: ",xyz)
                broadcast_once(xyz,"map","target-"+str(len(target_list)))
    whole_body.move_to_joint_positions({"head_tilt_joint":0,'head_pan_joint': 0})
    return target_list

def get_tfs(target_vector,frame_id = "head_rgbd_sensor_link",name="target"):
    '''Generate the tfs needed for broadcast'''
    broadcaster = tf2_ros.TransformBroadcaster()
    tfs = TransformStamped()
    tfs.header.frame_id = frame_id
    tfs.child_frame_id = name
    tfs.transform.translation.x = target_vector[0]
    tfs.transform.translation.y = target_vector[1]
    tfs.transform.translation.z = target_vector[2]
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    return broadcaster,tfs
def detect_once_broadcast(target_id,times=1):
    '''Detect once (repeat "times" times if not found) and keep broadcasting'''
    # Find vector
    for i in range(times):
        target_list = detect(target_id)
        if len(target_list)!=0:
            break
        rospy.sleep(0.3)
    if len(target_list)==0:
        return
    target_vector = get_vector(target_list[0])
    # print("Target details: ",target_box,target_vector,target_distance)
    # Broadcast it
    broadcaster,tfs = get_tfs(target_vector)
    signal.signal(signal.SIGINT, sig_int_handler)
    while True:
        tfs.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(tfs)
        rospy.sleep(0.2)
def detect_broadcast(target_id):
    '''Keep detecting and broadcasting'''
    signal.signal(signal.SIGINT, sig_int_handler)
    while True:
        # Find vector
        target_list = detect(target_id)
        if len(target_list)==0:
            rospy.sleep(0.2)
            continue
        target_vector = get_vector(target_list[0])
        # print("Target details: ",target_box,target_vector,target_distance)
        # Broadcast it
        broadcaster,tfs = get_tfs(target_vector)
        tfs.header.stamp = rospy.Time.now()
        rospy.sleep(0.2)
        try:
            broadcaster.sendTransform(tfs)
        except:
            continue
def broadcast_once(target_vector,frame_id,name):
    '''Broadcast a point for 1 sec for debug'''
    broadcaster,tfs = get_tfs(target_vector,frame_id,name)
    for i in range(10):
        tfs.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(tfs)
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node("detect")
    # print(detect(41,1))
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import my_functions
    spawn_position = [0,0,0]
    target_list = detect_all(41)
    print([my_functions.my_transform_can_yolo_list(i,spawn_position) for i in target_list])
