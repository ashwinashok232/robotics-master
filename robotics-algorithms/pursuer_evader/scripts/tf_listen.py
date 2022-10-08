#!/usr/bin/env python

## Code is based on ROS Documentation (ROS tf Tutorials) 2.1.1 - Writing a tf broadcaster (Python)
#  (http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29)

  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg


if __name__ == '__main__':
    
    rospy.init_node('tf_robot')
    listener = tf.TransformListener()
    robot_vel = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rate = rospy.Rate(10.0)
    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransformFull('/robot_1', rospy.Time.now(), '/robot_0', 
                                                       rospy.Time.now()-rospy.Duration(1.0), "/world")

        except (tf.LookupException, tf.ConnectivityException, tf.Exception):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        robot_vel.publish(cmd)

        rate.sleep()




