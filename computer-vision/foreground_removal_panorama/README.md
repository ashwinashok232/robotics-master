# Foreground Removal and Panorama

## Description
Task 1 - Foreground detection using ORB features and brute force matching. Perspective transformation is done to overlap foreground region of one image.

Task 2 - Images are comapred using ORB feature matching to find adjacent images (highest overlap). Finally, the images are warped by doing a perspective transformation based on their homography to be aligned with one another, and are stitched together.

## Outputs
t1.py outputs stitched images t1_1.png and t1_2.png (in images folder) with the foreground of t1_t.png removed.

t2.py outputs panorama of t2_<n>.png images. Images folder contains 6 images, but more can be used.
