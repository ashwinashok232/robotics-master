#!/usr/bin/env python

## Code is based on ROS Documentation (ROS tutorials) 1.1.12 - Writing a Simple Publisher and Subscriber (Python)
#  (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

import rospy
import numpy
from sensor_msgs.msg import LaserScan		
from geometry_msgs.msg import Twist, Vector3
import random
import math

rotation = False 	# defining global variable for rotation: False = straight motion, True = turning

def callback(data):
    for wall_dist in data.ranges:
        if wall_dist<=1: 	# conditional to trigger rotation if wall is less than/equal to one unit away
            global rotation
            rotation = True 

    
def evader_controller():
    
    global rotation
    pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=1) 
    rospy.init_node('evader')
    rospy.Subscriber('robot_0/base_scan', LaserScan, callback)
    rate = rospy.Rate(10)

# Establishing movement variables for straight (linear) motion and turning near walls
    straight = Twist()
    straight.linear.x = 2
    straight.linear.y = 0
    straight.linear.z = 0
    straight.angular.x = 0
    straight.angular.y = 0
    straight.angular.z = 0
    
    turn = Twist()
    turn.linear.x = 0
    turn.linear.y = 0
    turn.linear.z = 0
    turn.angular.x = 0
    turn.angular.y = 0

    while not rospy.is_shutdown():
        if not rotation: 	# conditional for choice between straight motion and turning near wall
            pub.publish(straight)
        else:
            turn.angular.z = 0.5*math.pi*random.randint(1,5) # z-component of turn defined under loop to allow random rotation choice
            pub.publish(turn)
            rate.sleep()
            rotation = False  	# rotation set to false to repeat straight motion         
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        evader_controller()
    except rospy.ROSInterruptException:
        pass

 
        
