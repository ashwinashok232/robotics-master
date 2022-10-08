import glob
import json
import os
import cv2
import sys
import face_recognition
import random
import numpy as np

def face_detection(path):
    json_list = []
    pathnames = []
    filenames = []

    pathnames = sorted(glob.glob(path),key=len)
    filenames = [list(pathnames[i].split('/'))[-1] for i in range(len(pathnames))]
                
    pathnames_sorted = sorted(pathnames,key= lambda i: int(i.split("/")[-1].split(".")[0]))
    filenames_sorted = [list(pathnames_sorted[i].split('/'))[-1] for i in range(len(pathnames_sorted))]
    print(filenames_sorted)
    
    
    boxes = []
    img_list = []
    img_crop = []
    for i in range(len(filenames_sorted)):
        face_cascade = cv2.CascadeClassifier('Model_Files/haarcascade_frontalface_default.xml')
        img = cv2.imread(pathnames_sorted[i])
        img_list.append(img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 4)
        for (x, y, w, h) in faces:
            #cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            element_1 = {"iname": filenames_sorted[i], "bbox": [float(x), float(y), float(w), float(h)]} #Image and bounding box for one face
            json_list.append(element_1) #JSON text for one image
            boxes.append([(y,x+w,y+h,x)]) #List of bounding boxes
            img_crop.append(img[y:y+h, x:x+w])

    return filenames_sorted, img_list, boxes, img_crop, json_list


def json_detection(json_list): #Dumps to JSON file
    output_json = "result.json"
    with open(output_json, 'w') as f:
        json.dump(json_list, f)

def json_cluster(json_list):
    output_json = "clusters.json"
    with open(output_json, 'w') as f:
        json.dump(json_list, f)    
    
def k_means_random(): #Chooses K random faces from encodings as cluster centers
    K = int(path_arg.split('_')[-1]) #int(''.join(sys.argv[1:])[-1])
    centers = []
    c_list = []
    c1 = np.random.randint(len(encodings))
    c_list.append(c1)
    centers.append(encodings[c1])
    
    
    while len(c_list) < K:
        dist_lst = []
        for i in range(len(encodings)):
            x_i = np.array(encodings[i])
            min_d = 10**10
            for j in range(len(centers)):
                c_i = np.array(centers[j])
                min_d = min(min_d, np.linalg.norm(x_i-c_i))
            dist_lst.append(min_d)          
        dist_lst = np.array(dist_lst)
        c_n = encodings[np.argmax(dist_lst)]
        centers.append(c_n)
        c_list.append(np.argmax(dist_lst))
        dist_lst = []
    
    return centers,K, c_list

def k_means_distance(encodings,centers): #Computes Euclidean distance between each cluster center and all faces
    dist = np.zeros((len(encodings),len(centers))) #Dist matrix: row = face encodings, col = cluster center number
    for i in range(len(centers)):
        for j in range(len(encodings)):
            dist[j,i] = np.linalg.norm(np.array(centers[i])-np.array(encodings[j]))
    return dist,centers

def k_means_center(dist,centers,K):
    centers_new = []
    if np.array_equal(np.unique(np.argmin(dist,axis=1)),np.arange(K)): #If there are fewer than K clusters, end loop and choose new random centers
        for k in range(K):
            cluster_sum = np.array(centers[k])
            count=0
            for l in np.where(np.argmin(dist,axis=1) == k)[0]:
                cluster_sum += np.array(encodings[l])
                count +=1
            cluster_average = cluster_sum/count
            end_loop = False
            centers_new.append(cluster_average)

    else:
        end_loop = True

            
    return centers_new,end_loop

def montage(final_cluster):
    for i in final_cluster:
        if len(i) == 1:
            montage = cv2.resize(np.array(i[0]),(100,100))
        elif len(i)%3==0:
            for j in range(0,len(i),3):
                img1 = cv2.resize(np.array(i[j]),(100,100))
                img2 = cv2.resize(np.array(i[j+1]),(100,100))
                img3 = cv2.resize(np.array(i[j+2]),(100,100))
                row = np.hstack([img1,img2,img3])
                if j == 0:
                    montage = row
                else:
                    montage = np.vstack([montage,row])
    
        elif (len(i)-1)%3==0:
            for j in range(0,len(i)-1,3):
                img1 = cv2.resize(np.array(i[j]),(100,100))
                img2 = cv2.resize(np.array(i[j+1]),(100,100))
                img3 = cv2.resize(np.array(i[j+2]),(100,100))
                row = np.hstack([img1,img2,img3])
                if j == 0:
                    montage = row
                else:
                    montage = np.vstack([montage,row])
            img1 = cv2.resize(np.array(i[-1]),(100,100))
            img2,img3 = np.zeros_like(img1)
            row = np.hstack([img1,img2,img3])
            montage = np.vstack([montage,row])
            
        elif (len(i)-2)%3==0:
            for j in range(0,len(i)-2,3):
                img1 = cv2.resize(np.array(i[j]),(100,100))
                img2 = cv2.resize(np.array(i[j+1]),(100,100))
                img3 = cv2.resize(np.array(i[j+2]),(100,100))
                row = np.hstack([img1,img2,img3])
                if j == 0:
                    montage = row
                else:
                    montage = np.vstack([montage,row])
            img1 = cv2.resize(np.array(i[-2]),(100,100))
            img2 = cv2.resize(np.array(i[-1]),(100,100))
            img3 = np.zeros_like(img1)
            row = np.hstack([img1,img2,img3])
            montage = np.vstack([montage,row])
        cv2.imshow('montage',montage)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
            



#path_arg = str(sys.argv)
path_arg = path_arg = sys.argv[1] #'Project3_data/FaceCluster_5'
if '~' in path_arg:
    path_arg = ''.join(sys.argv[1].split("~"))
path = path_arg+'/*'
print(path)
filenames_sorted, img_list, boxes, img_crop, json_list = face_detection(path)

encodings = [] #Stores 128-long encoding vectors for all faces
for i in range(len(img_list)):
    encodings.append(face_recognition.face_encodings(img_list[i], boxes[i]))



centers,K,randomizer = k_means_random()
dist,centers = k_means_distance(encodings,centers)
centers_new,end_loop = k_means_center(dist,centers,K)
cluster_old = np.zeros_like(np.argmin(dist,axis=1))
end_loop = False

while not np.array_equal(cluster_old,np.argmin(dist,axis=1)): #Repeat k-means until clusters do not change
    cluster_old = np.argmin(dist,axis=1) #cluster_old shows how face encodings were classified in the previous loop
    dist,centers = k_means_distance(encodings,centers_new)
    centers_new,end_loop = k_means_center(dist,centers,K)
    
    if end_loop:
        centers_new,K,randomzier = k_means_random()

        
cluster_list = np.argmin(dist,axis=1)
print(cluster_list)
final_cluster = []
json_list = []
for k in range(K):
    cluster = []
    file_lst = []
    for i in np.where(cluster_list==k)[0]:
        cluster.append(img_crop[i])
        file_lst.append(filenames_sorted[i])
    element_1 = {"cluster_no": k, "elements": file_lst}
    json_list.append(element_1)
    final_cluster.append(cluster)
    
json_cluster(json_list)
#montage(final_cluster) --> Used to generate montage