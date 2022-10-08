# 1. Only add your code inside the function (including newly improted packages). 
#  You can design a new function and call the new function in the given functions. 
# 2. For bonus: Give your own picturs. If you have N pictures, name your pictures such as ["t3_1.png", "t3_2.png", ..., "t3_N.png"], and put them inside the folder "images".
# 3. Not following the project guidelines will result in a 10% reduction in grades

import cv2
import numpy as np
import matplotlib.pyplot as plt
import json


def stitch(imgmark, N, savepath=''): #For bonus: change your input(N=*) here as default if the number of your input pictures is not 4.
    "The output image should be saved in the savepath."
    "The intermediate overlap relation should be returned as NxN a one-hot(only contains 0 or 1) array."
    "Do NOT modify the code provided."
    imgpath = [f'./images/{imgmark}_{n}.png' for n in range(1,N+1)]
    imgs = []
    name_list = []
    for ipath in imgpath:
        img = cv2.imread(ipath)
        imgs.append(img)
        name_list.append(ipath)

    orb = cv2.ORB_create(nfeatures=2000)
    kp_list = []
    des_list = []
    
    for image in range(len(imgs)):
        kp,des = orb.detectAndCompute(imgs[image],None)
        kp_list.append(kp)
        des_list.append(des)
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
    img_num1 = 0
    img_num2 = 0
    tracker = []
    match_list_full = np.zeros((1,3),dtype='int')
    
    
    for i in range(len(des_list)):
        des1 = des_list[i]
        for j in range(len(des_list)):
            des2 = des_list[j]
            if i!=j:
                good = 0
                for k in des1:
                    match_sum = 0
                    diff = k-des2
                    lst = np.sqrt(np.sum(np.square(diff),axis=1))
                    lst_sort = np.sort(lst,axis=0)
                    if lst_sort[0] < 0.6*lst_sort[1]:
                        good += 1
                if (j,i) not in tracker and good > 10:
                    tracker.append((i,j))
                    match_list_full = np.vstack((match_list_full,[[i,j,good]])).astype(int)
    match_list_full = np.delete(match_list_full,0,0)
    match_list = match_list_full[match_list_full[:,2].argsort()[::-1]]
        
        
    def initialization(name1,name2):
        img1 = cv2.imread(name1)
        img2 = cv2.imread(name2)
        return img1,img2
    
    def feature_matcher_old(img1,img2):
        orb = cv2.ORB_create(nfeatures=2000)
        
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
    
        bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)
        matches = bf.knnMatch(des1, des2,k=2)
        
        all_matches = []
        for m, n in matches:
            all_matches.append(m)
            
        good = []
        for m, n in matches:
            if m.distance < 0.8 * n.distance:
                good.append(m)
        return kp1,kp2,good
    
    
    def feature_matcher_new(img1,img2):
        orb = cv2.ORB_create(nfeatures=2000)
        
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        good = []
        index = 0
        for i in des1:
            diff = i-des2
            lst = np.sqrt(np.sum(np.square(diff),axis=1))
            lst_sort = np.sort(lst,axis=0)
    
            if lst_sort[0] < 0.7*lst_sort[1]:
                good.append((index,np.argmin(lst)))
            
            index+=1
        
        return kp1,kp2,good
    
    def warpImages(img1, img2, H):
    
        img1_pts = np.float32([[0,0], [0, img1.shape[0]],[img1.shape[1], img1.shape[0]], [img1.shape[1], 0]]).reshape(-1, 1, 2)
        img2_pts = np.float32([[0,0], [0,img2.shape[0]], [img2.shape[1],img2.shape[0]], [img2.shape[1],0]]).reshape(-1,1,2)
    
        img1_trans = cv2.perspectiveTransform(img2_pts, H)
        
        x_max = int(max(np.amax(img1_pts,axis=0)[0,0], np.amax(img1_trans,axis=0)[0,0]))
        y_max = int(max(np.amax(img1_pts,axis=0)[0,1], np.amax(img1_trans,axis=0)[0,1]))
        x_min = int(min(np.amin(img1_pts,axis=0)[0,0], np.amin(img1_trans,axis=0)[0,0]))
        y_min = int(min(np.amin(img1_pts,axis=0)[0,1], np.amin(img1_trans,axis=0)[0,1]))
        
        translation_dist = [-x_min,-y_min]
    
        H_trans = np.zeros((3,3),int)
        np.fill_diagonal(H_trans,1)
        H_trans[0,2] = -x_min
        H_trans[1,2] = -y_min  
        H = H_trans.dot(H)
        
        H_translation = np.array([[1, 0, translation_dist[0]], [0, 1, translation_dist[1]], [0, 0, 1]])
        output_img = cv2.warpPerspective(img2, H, (x_max-x_min, y_max-y_min))
        output_img[-y_min:img1.shape[0]-y_min, -x_min:img1.shape[1]-x_min] = img1
        return output_img
    
    def perspective_transform(good,kp1,kp2):
    
        MIN_MATCH_COUNT = 0
        
        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m[0]].pt for m in good]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m[1]].pt for m in good]).reshape(-1,1,2)

        M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        return M
            
    
    overlap_arr = np.zeros((len(name_list),len(name_list)))
    np.fill_diagonal(overlap_arr,1)
    
    
    img1 = cv2.imread(name_list[match_list[0,0]])
    img2 = cv2.imread(name_list[match_list[0,1]]) 
    completed_matches = [match_list[0,0],match_list[0,1]]
    kp1,kp2,good = feature_matcher_new(img1,img2)
    M = perspective_transform(good,kp1,kp2)
    result = warpImages(img2, img1, M)
    
    
    overlap_arr[match_list[0,0],match_list[0,0]] = 1
    overlap_arr[match_list[0,0],match_list[0,1]] = 1
    overlap_arr[match_list[0,1],match_list[0,0]] = 1
    overlap_arr[match_list[0,1],match_list[0,1]] = 1
    
    
    i = 1
    while len(completed_matches) < len(name_list):
        img1 = result
        if match_list[i,0] not in completed_matches and match_list[i,1] in completed_matches:
            img2 = cv2.imread(name_list[match_list[i,0]])
            completed_matches.append(match_list[i,0])
            overlap_arr[match_list[i,0],match_list[i,0]] = 1
            overlap_arr[match_list[i,0],match_list[i,1]] = 1
            overlap_arr[match_list[i,1],match_list[i,0]] = 1
            overlap_arr[match_list[i,1],match_list[i,1]] = 1
            
            i += 1
        elif match_list[i,1] not in completed_matches and match_list[i,0] in completed_matches:
            img2 = cv2.imread(name_list[match_list[i,1]])
            completed_matches.append(match_list[i,1])
            overlap_arr[match_list[i,0],match_list[i,0]] = 1
            overlap_arr[match_list[i,0],match_list[i,1]] = 1
            overlap_arr[match_list[i,1],match_list[i,0]] = 1
            overlap_arr[match_list[i,1],match_list[i,1]] = 1
            i += 1
        else:
            i+=1
            continue
        
        kp1,kp2,good = feature_matcher_new(img1,img2)
        M = perspective_transform(good,kp1,kp2)
        result = warpImages(img2, img1, M)
    
    if i-1 < match_list.shape[0]:
        for j in range(i,match_list.shape[0]):
            overlap_arr[match_list[j,0],match_list[j,0]] = 1
            overlap_arr[match_list[j,0],match_list[j,1]] = 1
            overlap_arr[match_list[j,1],match_list[j,0]] = 1
            overlap_arr[match_list[j,1],match_list[j,1]] = 1
            
    cv2.imwrite(savepath,result)

    return overlap_arr

if __name__ == "__main__":
    #task2
    overlap_arr = stitch('t2', N=4, savepath='task2.png')
    with open('t2_overlap.txt', 'w') as outfile:
        json.dump(overlap_arr.tolist(), outfile)
    #bonus
    overlap_arr2 = stitch('t3', N=5, savepath='task3.png')
    with open('t3_overlap.txt', 'w') as outfile:
        json.dump(overlap_arr2.tolist(), outfile)
