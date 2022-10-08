import glob
import json
import os
import cv2
import sys

json_list = []
pathnames = []
filenames = []

#path_arg = str(sys.argv)
#path_arg = 'Project3_data/Validation folder/images'
#path = path_arg+'/*'


def face_detection(path):
    pathnames = sorted(glob.glob(path),key=len)
    filenames = [list(pathnames[i].split('/'))[-1] for i in range(len(pathnames))]    
    
    pathnames_sorted = sorted(pathnames,key= lambda i: int(i.split("/")[-1].split("_")[-1].split(".")[0]))
    filenames_sorted = [list(pathnames_sorted[i].split('/'))[-1] for i in range(len(pathnames_sorted))]
    print(filenames_sorted)
    
    boxes_dict = {}
    boxes = []
    img_list = []
    for i in range(len(filenames)):
        face_cascade = cv2.CascadeClassifier('Model_Files/haarcascade_frontalface_default.xml')#('haarcascade_profileface.xml')
        img = cv2.imread(pathnames_sorted[i])
        img_list.append(img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 4)#(gray, 1.3, 5)#
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            element_1 = {"iname": filenames_sorted[i], "bbox": [float(x), float(y), float(w), float(h)]} #first element in json file
            json_list.append(element_1)
            boxes.append([(y,x+w,y+h,x)])
        boxes_dict[str(i)] = boxes
        boxes = []
        cv2.imshow('image',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return json_list


path_arg = sys.argv[1]
if '~' in path_arg:
    path_arg = ''.join(sys.argv[1].split("~"))
path = path_arg + '/images/*'
print(path)


face_detection(path)

output_json = "results.json"
with open(output_json, 'w') as f:
    json.dump(json_list, f)

