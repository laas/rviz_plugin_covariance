#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_plugin_covariance' )
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from math import cos, sin
import tf

topic = 'test_covariance'
publisher = rospy.Publisher( topic, PoseWithCovarianceStamped )

rospy.init_node( 'test_covariance' )

br = tf.TransformBroadcaster()
rate = rospy.Rate(10)
radius = 1
angle = 0
r = 0
p = 0
y = 0

while not rospy.is_shutdown():

    cov = PoseWithCovarianceStamped()
    cov.header.frame_id = "/base_link"
    cov.header.stamp = rospy.Time.now()
   
    cov.pose.pose.position.x = cos( 10 * angle )
    cov.pose.pose.position.y = sin( 20 * angle )
    cov.pose.pose.position.z = sin( 40 * angle )

    cov.pose.pose.orientation.w = 1.0
    cov.pose.pose.orientation.x = 0.0
    cov.pose.pose.orientation.y = 0.0
    cov.pose.pose.orientation.z = 0.0

    cov.pose.covariance[0] = 4+2*sin(20*angle)
    cov.pose.covariance[6+1] = 4+2*sin(20*angle)
    cov.pose.covariance[12+2] = 1+0.5*sin(20*angle)
    cov.pose.covariance[18+3] = 0.01
    cov.pose.covariance[24+4] = 0.01
    cov.pose.covariance[30+5] = 0.01

    publisher.publish( cov )

    br.sendTransform((radius * cos(angle), radius * sin(angle), 0),
                     tf.transformations.quaternion_from_euler(r, p, y),
                     rospy.Time.now(),
                     "base_link",
                     "map")
    angle += .01
    r = angle
    p = angle
    y = angle
    rate.sleep()

