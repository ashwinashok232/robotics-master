#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math
import random
import numpy as np
import sys

def callback(data):
    ranges = data.ranges
    angle_min = data.angle_min
    angle_inc = data.angle_increment
    laser_to_cartesian(ranges,angle_min,angle_inc)

def laser_to_cartesian(ranges,angle_min,angle_inc):
    i = 0
    pts = []
    for r in ranges:
        i = i+1
        if r<3:
            t = angle_min + i * angle_inc
            px = r*math.cos(t)
            py = r*math.sin(t)
            pz = 0
            pts.append((px,py,pz))
    ransac(pts)


def ransac(pts):
    point_thres = 30
    k = 30
    thres = 0.01
    ransac_line = []

    
    #Loop for outliers to generate all possible lines given thresholds
    while len(pts) > point_thres:

        max_inlier_count = 0
        final_bin = []
        p1 = Point()
        p2 = Point()
        
        #Loop for k-iterations to find one line based on inliers
        for iteration in range(k):
       
            #Random Point Generation        
            rand1 = random.randint(0,len(pts)-1)
            rand2 = random.randint(0,len(pts)-1)

            if rand1 == rand2:
                rand2 = random.randint(0,len(pts)-1)

            x1 = pts[rand1][0]
            y1 = pts[rand1][1]
            x2 = pts[rand2][0]
            y2 = pts[rand2][1]
        

            #Line Model
            if x2-x1 != 0:
                m = (y2-y1)/(x2-x1+sys.float_info.epsilon)
                c = y1 - m*x1
            else:
                break


            #Inlier/Outlier Bins
            inlier_count = 0
            in_out_bin = []
            for i in range(len(pts)):
                dist = abs((m*pts[i][0] - pts[i][1] + c)) / math.sqrt(m**2 + 1)

                if  dist < thres:
                    in_out_bin.append(1)
                    inlier_count += 1
                else:
                    in_out_bin.append(0)


            if max_inlier_count < inlier_count:
                max_inlier_count = inlier_count
                final_bin = in_out_bin


        #Fidning optimal line of bets fit among inliers 
        max_inlier_dist = 0 

        for j in range(len(final_bin)):
            if final_bin[j] == 1: 
                for l in range(len(final_bin)):
                    if final_bin[l] != 0:
                        inlier_dist_x = (pts[j][0] - pts[l][0])**2
                        inlier_dist_y = (pts[j][1] - pts[l][1])**2
                        inlier_dist = (inlier_dist_x + inlier_dist_y)**0.5
                        if inlier_dist > max_inlier_dist:
                            max_inlier_dist = inlier_dist
                            p1 = Point(pts[j][0], pts[j][1], 0)
                            p2 = Point(pts[l][0], pts[l][1], 0)

        ransac_line.append(p1)
        ransac_line.append(p2)        

        #Removing inliers to generate second line (if applicable) 
        loop_count = 0     
        for n in range(len(final_bin)):
            if final_bin[n] == 1:
                pts.pop(n-loop_count)
                loop_count = loop_count+ 1

    #print(len(ransac_line))            
    marker.points = ransac_line
    pub.publish(marker)


if __name__ == '__main__':

        rospy.init_node('rvizmark')
        rate = rospy.Rate(10)
        pub = rospy.Publisher("marker", Marker, queue_size=10)
        marker= Marker()
        marker.header.frame_id="/odom"
        marker.ns = 'lines'
        marker.action = marker.ADD
        #marker.lifetime = rospy.Duration(1.0) 
        marker.type = marker.LINE_LIST
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1 
        marker.color.a = 1.0
        marker.color.r = 1.0  

        rospy.Subscriber("/base_scan", LaserScan, callback)

        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.2)

	    #perception()
    #except rospy.ROSInterruptException:
     #   pass
