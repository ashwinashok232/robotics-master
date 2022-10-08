#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import random
import sys
import tf


def base_scan(laser_msg):
    global ranges, intensities
    ranges = laser_msg.ranges
    intensities = laser_msg.intensities


def path_line_tracker(m,c,robot_loc):
    global cross_prev
    if robot_loc[1] <= (m * robot_loc[0] + c):
        cross_now = -1
    else:
        cross_now = 1
    if cross_now != cross_prev:
        cross_prev = cross_now
        return True
    else:
        return False


def pose_broadcaster(msg):
    
    global goal_loc, robot_loc
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     'robot',
                     'world')

    br.sendTransform((goal_loc[0], goal_loc[1], 0),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     'goal',
                     'world')

    robot_loc[0] = msg.pose.pose.position.x
    robot_loc[1] = msg.pose.pose.position.y


def wall_follow(ranges, intensities):
    global linVel, rotVel
    intensity_count = 0
    for i in intensities[180:360]:
        if i == 1:
            intensity_count += 1

    if intensity_count > intensity_max:
        linVel = 0
        rotVel = -20
    elif intensity_count < intensity_min:
        linVel = 0
        rotVel = 20
        if min(ranges[250:360])>2.95 and min(ranges[250:360])<2.99:
            linVel = 2
            rotVel = 0
        elif min(ranges[250:360])>=2.99:
            linVel = 0
            rotVel = 20
    else:
        linVel = 2
        rotational_vel = 0
    return linVel, rotVel





if __name__ == '__main__':

    rospy.init_node('bug2', anonymous=True)
    bug_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/base_scan', LaserScan, base_scan)
    rospy.Subscriber('/base_pose_ground_truth', Odometry, pose_broadcaster)
    
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    move = Twist()
    state = 'GOALSEEK'
    robot_loc = [-8.0,-2.0,0] #x,y,angle
    goal_loc = [4.5,9.0]
    reached_goal = 0
    goal_error = 1.75
    m = 0.88
    c = 5.04
    cross_prev = 1
    intensity_min = 85
    intensity_max = 140
    ranges = [3]*361
    intensities = [0]*361
    trans = [5,5,0]
    rotation_tracker = 0
    rate = rospy.Rate(20)

    


    while not rospy.is_shutdown():

        if reached_goal != 0:
            print('GOAL REACHED (WITHIN THRESHOLD)')
        else:
            goaldist_x = (robot_loc[0] - goal_loc[0])**2
            goaldist_y = (robot_loc[1] - goal_loc[0])**2
            goaldist = (goaldist_x + goaldist_y)**0.5
            linVel = 0
            rotVel = 0

            if goaldist > goal_error: 
                if state == 'GOALSEEK':
                    try: 
                        (trans,rot) = listener.lookupTransform('/robot', '/goal', rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        pass
                    
                    if abs(math.atan2(trans[1],trans[0]))<0.05:
                        linVel = 2
                        rotVel = 0
                        rotation_tracker = 0
                    else:
                        linVel = 0
                        rotVel = 1
                        rotation_tracker = 1

                    if (min(ranges[150:210])<0.8) and  rotation_tracker == 0:
                        state = 'WALLFOLLOW'
                        rotation_tracker = 0
                        linVel,rotVel = wall_follow(ranges, intensities)
                        
                
                elif state == 'WALLFOLLOW':
                    linVel,rotVel = wall_follow(ranges, intensities)
                    
                    if path_line_tracker(m, c, robot_loc):
                        state = 'GOALSEEK'
                move.linear.x = linVel
                move.angular.z = rotVel
                bug_pub.publish(move)
            else:
                linVel = 0
                rotVel = 0
                state = 'GOALREACHED'
                reached_goal = 1
        rate.sleep()
                    
            


            




    


