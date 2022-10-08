"""
Character Detection

The goal of this task is to implement an optical character recognition system consisting of Enrollment, Detection and Recognition sub tasks

Please complete all the functions that are labelled with '# TODO'. When implementing the functions,
comment the lines 'raise NotImplementedError' instead of deleting them.

Do NOT modify the code provided.
Please follow the guidelines mentioned in the project1.pdf
Do NOT import any library (function, module, etc.).
"""


import argparse
import json
import os
import glob
import cv2
import numpy as np


def read_image(img_path, show=False):
    """Reads an image into memory as a grayscale array.
    """
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    if show:
        show_image(img)

    return img

def show_image(img, delay=1000):
    """Shows an image.
    """
    cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('image', img)
    cv2.waitKey(delay)
    cv2.destroyAllWindows()

def parse_args():
    parser = argparse.ArgumentParser(description="cse 473/573 project 1.")
    parser.add_argument(
        "--test_img", type=str, default="./data/test_img.jpg",
        help="path to the image used for character detection (do not change this arg)")
    parser.add_argument(
        "--character_folder_path", type=str, default="./data/characters",
        help="path to the characters folder")
    parser.add_argument(
        "--result_saving_directory", dest="rs_directory", type=str, default="./",
        help="directory to which results are saved (do not change this arg)")
    args = parser.parse_args()
    return args

def ocr(test_img, characters):
    """Step 1 : Enroll a set of characters. Also, you may store features in an intermediate file.
       Step 2 : Use connected component labeling to detect various characters in an test_img.
       Step 3 : Taking each of the character detected from previous step,
         and your features for each of the enrolled characters, you are required to a recognition or matching.

    Args:
        test_img : image that contains character to be detected.
        characters_list: list of characters along with name for each character.

    Returns:
    a nested list, where each element is a dictionary with {"bbox" : (x(int), y (int), w (int), h (int)), "name" : (string)},
        x: row that the character appears (starts from 0).
        y: column that the character appears (starts from 0).
        w: width of the detected character.
        h: height of the detected character.
        name: name of character provided or "UNKNOWN".
        Note : the order of detected characters should follow english text reading pattern, i.e.,
            list should start from top left, then move from left to right. After finishing the first line, go to the next line and continue.
        
    """
    # TODO Add your code here. Do not modify the return and input arguments

    enrollment()

    detection()
    
    recognition()

    raise NotImplementedError

def enrollment():
    """ Args:
        You are free to decide the input arguments.
    Returns:
    You are free to decide the return.
    """
    # TODO: Step 1 : Your Enrollment code should go here.

    def image_loader(folder):
        image_list = []
        for filename in os.listdir(folder):
            img = cv2.imread(os.path.join(folder,filename))
            img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)[1]
    
            if img is not None:
                image_list.append(img)
        return image_list


    def sift_function(image_list):
        kp_sample = []
        des_sample = []
        sift = cv2.SIFT_create()
        
        for i in range(len(image_list)):
            keypoints,descriptors = sift.detectAndCompute(image_list[i],None)
            if keypoints is not None and descriptors is not None:
                kp_sample.append(keypoints)
                des_sample.append(descriptors)
        
        return kp_sample,des_sample


    image_list = image_loader('./data/characters')
    kp_sample,des_sample = sift_function(image_list)
    return kp_sample,des_sample,image_list
    
    raise NotImplementedError

def detection(img):
    """ 
    Use connected component labeling to detect various characters in an test_img.
    Args:
        You are free to decide the input arguments.
    Returns:
    You are free to decide the return.
    """
    # TODO: Step 2 : Your Detection code should go here.
    
    img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)[1]
    
    def first_pass(img):
        counter = 1
        img_label = np.zeros((img.shape[0],img.shape[1]))
        union = []
        
        for row in range(img.shape[0]):
            for col in range(img.shape[1]):
                if img[row,col] == 255: #Foreground pixels
                    if row == 0: #First row
                        if col == 0: #First pixel white
                            img_label[row,col] = counter
                            counter += 1
                            
                        elif img[row,col-1] == 255: #Left neighbour white in first row
                            img_label[row,col] = img_label[row,col-1]
                            
                        else: #Left neighbour black in first row
                            img_label[row,col] = counter
                            counter += 1
                            
                    elif col == 0: #First column but not first row 
                        if img[row-1,col] == 255: #Above neighbour white
                            img_label[row,col] = img_label[row-1,col]
        
                        else: #Above neighbour black
                            img_label[row,col] = counter
                            counter += 1
                            
                    elif img[row,col-1] == 255 and img[row-1,col] == 255: #Both left and above neighbors white
                        child = max(img_label[row,col-1],img_label[row-1,col])
                        parent = min(img_label[row,col-1],img_label[row-1,col])
                        img_label[row,col] =  parent #Lower label of both neighbours chosen                
                        
                        if (parent,child) not in union and parent != child:
                            union.append((parent,child))
                            
                            for j in range(len(union)-1):
                                if union[j][1] == parent:
                                    union[-1] = (union[j][0],child)
        
                    
                    elif img[row,col-1] == 255 and img[row-1,col] == 0: #Left neighbour white, above neighbour black
                        img_label[row,col] = img_label[row,col-1]
                        
                    elif img[row,col-1] == 0 and img[row-1,col] == 255: #Left neighbour black, above neighbour white
                        img_label[row,col] = img_label[row-1,col]
                        
                    elif img[row,col-1] == 0 and img[row-1,col] == 0: #Left neighbour black, above neighbour black
                        img_label[row,col] = counter
                        counter += 1
                         
                elif img[row,col] == 0: #Background pixel
                    img_label[row,col] = 0
                    
        return img_label,union
    
    def second_pass(img_label,union):
        for i in union: #connects all parent and child labels
            img_label = np.where(img_label == i[1],i[0],img_label)
        
        labels = np.unique(img_label)
        
        for j in range(len(labels)):
            img_label = np.where(img_label==labels[j],j,img_label)
        
        labels = np.unique(img_label)
    
        return img_label,labels
    
    
    def character_detection(labels,img_label):
        for i in range(len(labels)):
            if i == 0:
                continue
            else:
                char_loc = (np.where(img_label==labels[i]))
                row_min = min(char_loc[0])
                row_max = max(char_loc[0])
                col_min = min(char_loc[1])
                col_max = max(char_loc[1])
                cv2.rectangle(img_label, (row_min, col_min), (row_max, col_max), (255,0,0), 2)
            
        return img_label
    
    def bounding_rectangles(img_input,sift):
        img_input = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)[1]
        count = 0
        kp2 = []
        des2 = []
    
        cont, h = cv2.findContours(img_input, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in cont:
            # get the bounding rect
            x1, y1, w, h = cv2.boundingRect(c)
            x2 = x1+w
            y2 = y1+h
            #if count==3:
            if x2< img_input.shape[0] and y2<img_input.shape[1]:
                img2 = img_input[x1:x2,y1:y2]
                width = int(img2.shape[1] * 1000 / 100)
                height = int(img2.shape[0] * 1000 / 100)
                img2 = cv2.resize(img2, (width,height), interpolation = cv2.INTER_AREA)
                keypoint, descriptor = sift.detectAndCompute(img2,None)
                if descriptor is not None:
                    kp2.append(keypoint)
                    des2.append(descriptor)
        
        return img2,kp2,des2
    
        #def colorize_ccl(img_label):
        #    label_hue = np.uint8(179*img_label/np.max(img_label))
        #    blank_ch = 255*np.ones_like(label_hue)
        #    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])
        #    
        #    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)
        #    labeled_img[label_hue==0] = 0#
        #    plt.imshow(cv2.cvtColor(labeled_img, cv2.COLOR_BGR2RGB))
        #    plt.axis('off')
        #    plt.title("Image after Component Labeling")
        #    plt.show()
    
    image_list = image_loader('data/characters')
    kp_sample,des_sample,sift = sift_function(image_list)
    img_label = character_detection(labels,img_label)
    colorize_ccl(img_label)
    img2,kp2,des2 = bounding_rectangles(img_input,sift)


    return img2,kp2,des2


   
def recognition(des_sample,des2,image_list):
    
    """ 
    Args:
        You are free to decide the input arguments.
    Returns:
    You are free to decide the return.
    """
    # TODO: Step 3 : Your Recognition code should go here.
    bf = cv2.BFMatcher()
    for i in des2:
        matches = bf.knnMatch(des_sample[2],i,k=2)
    match = []
    for m,n in matches:
        print(m.distance)

    raise NotImplementedError


def save_results(coordinates, rs_directory):
    """
    Donot modify this code
    """
    results = []
    with open(os.path.join(rs_directory, 'results.json'), "w") as file:
        json.dump(results, file)


def main():
    args = parse_args()
    
    characters = []

    all_character_imgs = glob.glob(args.character_folder_path+ "/*")
    
    for each_character in all_character_imgs :
        character_name = "{}".format(os.path.split(each_character)[-1].split('.')[0])
        characters.append([character_name, read_image(each_character, show=False)])

    test_img = read_image(args.test_img)

    results = ocr(test_img, characters)

    save_results(results, args.rs_directory)


if __name__ == "__main__":
    main()
