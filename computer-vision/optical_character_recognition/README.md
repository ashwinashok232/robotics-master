# Optical Character Recognition

## Description
Enrollment - Uses SIFT and edge detection for template matching

Detection - Manually implements connected component labelling (not using openCV directly) to detect individual characters in test image

Recognition - Uses brute force matching to compare detected and template characters

## Issues and Conclusions
Manual implementation of connected component labelling is noisy, using OpenCV would be more effective. SIFT is not an effective feature extraction method here since there are limited distinct features in each characters, so comparison is limited.
