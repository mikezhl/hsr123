# YOLO object detection test2: simply
import cv2
import numpy as np
import matplotlib.pyplot as plt

yolo = cv2.dnn.readNet("yolo_models/yolov3.weights", "yolo_models/yolov3.cfg")
classes = []
with open("yolo_models/coco.names","r") as file:
    classes = [line.strip() for line in file.readlines()]
layer_names = yolo.getLayerNames()
outputlayers = [layer_names[i -1] for i in yolo.getUnconnectedOutLayers()]
colorRed   = (0, 0, 255)
colorGreen = (0, 255, 0)
name = "images/3.jpg"
img = cv2.imread(name)
height, width, channels = img.shape
plt.imshow(img)
blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (608, 608), swapRB=True, crop=False)
yolo.setInput(blob)
outputs = yolo.forward(outputlayers)
class_ids = []
confidences = []
boxes = []
for output in outputs:
    for detection in output:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if (confidence > 0):
            print(confidence)
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)
            x = int(center_x - w/2)
            y = int(center_y - h/2)
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)
indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.001, 0.001)
colors = np.random.uniform(0, 255, size=(len(classes), 3))
for i in range(len(boxes)):
    if i in indices:
        x, y, w, h = boxes[i]
        label =str(classes[class_ids[i]])
        start = (x,y)
        end = (x+w,y+h)
        cv2.rectangle(img, start, end, (0,255,0), 4)
        cv2.putText(img, label, (x,y-20), cv2.FONT_HERSHEY_PLAIN, 5, colorRed, 4)
cv2.imshow('window', img)
cv2.waitKey(0)
# cv2.imwrite("images/output-v3-2.jpg", img)