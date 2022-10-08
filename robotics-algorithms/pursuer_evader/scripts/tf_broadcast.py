#!/usr/bin/env python

## Code is based on ROS Documentation (ROS tf Tutorials) 2.1.1 - Writing a tf broadcaster (Python)
# (http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29#CA-535ecf1b33de9267f08a855ee826b648436c3ded_11)

## Input arguments based on nav_msgs documentation (http://docs.ros.org/melodic/api/nav_msgs/html/index-msg.html)


import roslib
import rospy
import nav_msgs.msg
import tf


def handle_robot_pos(msg, robotname):

    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     robotname,
                     "world")

if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    robotname = rospy.get_param('~robot')
    rospy.Subscriber('/%s/base_pose_ground_truth' % robotname,
                     nav_msgs.msg.Odometry,
                     handle_robot_pos,
                     robotname)
    rospy.spin()
