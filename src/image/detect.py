import numpy as np
import cv2

from get_image import get_image
from get_distance import get_distance

def format_yolov5(frame):
        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result
def detect(target_id,debug=0):
    c=0.01
    # Load the model and feed a 640x640 image to get predictions
    net = cv2.dnn.readNet('yolo_models/yolov5s.onnx')
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
    with open("yolo_models/classes.txt", "r") as f:
        class_list = [cname.strip() for cname in f.readlines()]
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0, 0)
    result_class_ids = []
    result_confidences = []
    result_boxes = []
    target_list = []
    for i in indexes:
        print("Class: ",class_ids[i]," Confidence: ",confidences[i]," Position: ",boxes[i])
        result_confidences.append(confidences[i])
        result_class_ids.append(class_ids[i])
        result_boxes.append(boxes[i])
        if class_ids[i] == target_id:
            target_list.append([confidences[i],boxes[i]])
    if len(target_list)==0:
        print("Target not found")
    else:
        print("Target reuslt:\n",target_list)
    # Plot the result
    if debug:
        for i in range(len(result_class_ids)):
            box = result_boxes[i]
            class_id = result_class_ids[i]
            cv2.rectangle(image, box, (0, 255, 255), 2)
            cv2.rectangle(image, (box[0], box[1] - 20), (box[0] + box[2], box[1]), (0, 255, 255), -1)
            cv2.putText(image, class_list[class_id], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
        cv2.imshow("output", image)
        
    target_box = target_list[0][1]
    target_distance = get_distance(target_box,1)



    print(target_box,target_distance)


if __name__ == '__main__':
    detect(41,1)
    cv2.waitKey()