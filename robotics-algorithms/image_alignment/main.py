#!/usr/bin/env python
from PIL import Image
import numpy as np
import sys
import os
from pathlib import Path


def ssd(im_R,im_G,im_B,filename):
    ssd_G = 10**10
    ssd_B = 10**10

    for row in range (-15,15):
        for col in range(-15,15):
            ssd_G_new = 0
            ssd_B_new = 0
            for i in range(50,250):
                for j in range(50,300):
                    ssd_G_new = ssd_G_new+abs(im_R[i+row][j+col]-im_G[i][j])
                    ssd_B_new = ssd_B_new+abs(im_R[i+row][j+col]-im_B[i][j])
 
            if ssd_G > ssd_G_new:
                ssd_G = ssd_G_new
                disp_G_ssd = (row,col)

            if ssd_B > ssd_B_new:
                ssd_B = ssd_B_new
                disp_B_ssd = (row,col)

    print(filename,' R-G disp. (row,col) based on SSD: {}'.format(disp_G_ssd))
    print(filename,' R-B disp. (row,col) based on SSD: {}'.format(disp_B_ssd))

    im_G_ssd = np.roll(im_G,disp_G_ssd[0],axis=0)
    im_G_ssd = np.roll(im_G_ssd,disp_G_ssd[1],axis=1)
    im_B_ssd = np.roll(im_B,disp_B_ssd[0],axis=0)
    im_B_ssd = np.roll(im_B_ssd,disp_B_ssd[1],axis=1)

    im_RGB_ssd = np.dstack((im_R, im_G_ssd, im_B_ssd))
    im_ssd = Image.fromarray(np.uint8(im_RGB_ssd))
    im_ssd.save(filename+"-ssd.jpg")



def ncc(im_R,im_G,im_B,filename):
    ncc_G = 0
    ncc_B = 0

    norm_R = (im_R-np.average(im_R))/np.sum((im_R-np.average(im_R))**2)
    norm_G = (im_G-np.average(im_G))/np.sum((im_G-np.average(im_G))**2)
    norm_B = (im_B-np.average(im_B))/np.sum((im_B-np.average(im_B))**2)

    for row in range (-15,15):
        for col in range(-15,15):
            ncc_G_new = 0
            ncc_B_new = 0
            for i in range(50,260):
                for j in range(50,260):
                    ncc_G_new = ncc_G_new + (norm_R[i+row][j+col]*norm_G[i][j])
                    ncc_B_new = ncc_B_new + (norm_R[i+row][j+col]*norm_B[i][j])
            if ncc_G < ncc_G_new:
                ncc_G = ncc_G_new
                disp_G_ncc = (row,col)


            if ncc_B < ncc_B_new:
                ncc_B = ncc_B_new
                disp_B_ncc = (row,col)


    print(filename,'R-G disp. (row,col) based on NCC: {}'.format(disp_G_ncc))
    print(filename,' R-B disp. (row,col) based on NCC: {}'.format(disp_B_ncc))

    im_G_ncc = np.roll(im_G,disp_G_ncc[0],axis=0)
    im_G_ncc = np.roll(im_G_ncc,disp_G_ncc[1],axis=1)
    im_B_ncc = np.roll(im_B,disp_B_ncc[0],axis=0)
    im_B_ncc = np.roll(im_B_ncc,disp_B_ncc[1],axis=1)

    im_RGB_ncc = np.dstack((im_R, im_G_ncc, im_B_ncc))
    im_ncc = Image.fromarray(np.uint8(im_RGB_ncc))
    im_ncc.save(filename+"-ncc.jpg")       
        

        
if __name__ == '__main__':
    im_path = input('Image Path: ')
    for file in os.listdir(im_path):
        if file.endswith(".jpg"):
            im = np.array(Image.open(im_path+file))

            im_B = np.zeros((341,np.shape(im)[1]))
            for i in range(0,340):
                for j in range(0,np.shape(im)[1]):
                    im_B[i][j] = im[i][j]


            im_G = np.zeros((341,np.shape(im)[1]))
            for k in range(341,681):
                for l in range(0,np.shape(im)[1]):
                    im_G[k-341][l] = im[k][l]


            im_R = np.zeros((341,np.shape(im)[1]))
            for m in range(682,1023):
                for n in range(0,np.shape(im)[1]):
                    im_R[m-682][n] = im[m][n]


            im_RGB = np.dstack((im_R, im_G, im_B))
            new_im_RGB = Image.fromarray(np.uint8(im_RGB))
            filename = os.path.splitext(file)[0]
            new_im_RGB.save(filename+"-color.jpg")

            ssd(im_R,im_G,im_B,filename)
            ncc(im_R,im_G,im_B,filename)



