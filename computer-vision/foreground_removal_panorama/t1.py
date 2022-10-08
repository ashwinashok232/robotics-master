#Only add your code inside the function (including newly improted packages)
# You can design a new function and call the new function in the given functions. 
# Not following the project guidelines will result in a 10% reduction in grades

import cv2
import numpy as np
import matplotlib.pyplot as plt


def stitch_background(img1, img2, savepath=''):
    "The output image should be saved in the savepath."
    "Do NOT modify the code provided."
    
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
            
        H_trans = np.zeros((3,3),int)
        np.fill_diagonal(H_trans,1)
        H_trans[0,2] = -x_min
        H_trans[1,2] = -y_min  
        H = H_trans.dot(H)
        
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

    kp1,kp2,good = feature_matcher_new(img1,img2)
    M = perspective_transform(good,kp1,kp2)
    result = warpImages(img2, img1, M)
    cv2.imwrite(savepath,result)

    return

if __name__ == "__main__":
    img1 = cv2.imread('./images/t1_1.png')
    img2 = cv2.imread('./images/t1_2.png')
    savepath = 'task1.png'
    stitch_background(img1, img2, savepath=savepath)

