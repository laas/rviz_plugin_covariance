#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_plugin_covariance' )
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import rospy
from math import cos, sin, pi
import tf
import tf_conversions

publisher_cov = rospy.Publisher( 'pose_with_cov', PoseWithCovarianceStamped, queue_size=5 )
publisher_dyn_pos = rospy.Publisher( 'dyn_pos_pose', PoseStamped, queue_size=5 )
publisher_dyn_ori = rospy.Publisher( 'dyn_ori_pose', PoseStamped, queue_size=5 )

rospy.init_node( 'test_covariance' )

br = tf.TransformBroadcaster()
rate = rospy.Rate(100)
angle = 0

axes='sxyz' # 'sxyz' or 'rxyz'

r = pi/2 # 90 deg
p = pi/3 # 60 deg
y = 0

pos_deviation = 0.5;
ori_deviation = pi/6.0;
ori_deviation2 = pi/18.0;

while not rospy.is_shutdown():
    stamp = rospy.Time.now()

    # Define static pose with covariance
    pose_with_cov = PoseWithCovarianceStamped()
    pose_with_cov.header.frame_id = "/base_link"
    pose_with_cov.header.stamp = stamp

    pose_with_cov.pose.pose.position.x = 1
    pose_with_cov.pose.pose.position.y = 1
    pose_with_cov.pose.pose.position.z = 1

    ori = pose_with_cov.pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(r,p,y, axes)

    pose_with_cov.pose.covariance[0] = pos_deviation**2.0
    pose_with_cov.pose.covariance[6+1] = 0.001
    pose_with_cov.pose.covariance[12+2] = 0.001
    pose_with_cov.pose.covariance[18+3] = ori_deviation2**2.0
    pose_with_cov.pose.covariance[24+4] = ori_deviation2**2.0
    pose_with_cov.pose.covariance[30+5] = ori_deviation**2.0

    # Define a dynamic pose that should change position inside the deviation
    dyn_pos_pose = PoseStamped()
    dyn_pos_pose.header.frame_id = "/base_link"
    dyn_pos_pose.header.stamp = stamp

    dyn_pos_pose.pose.position.x = pose_with_cov.pose.pose.position.x + pos_deviation*cos( 10 * angle )
    dyn_pos_pose.pose.position.y = pose_with_cov.pose.pose.position.y
    dyn_pos_pose.pose.position.z = pose_with_cov.pose.pose.position.z

    ori = dyn_pos_pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(r,p,y, axes)

    # Define a dynamic pose that should change orientation inside the deviation
    dyn_ori_pose = PoseStamped()
    dyn_ori_pose.header.frame_id = "/base_link"
    dyn_ori_pose.header.stamp = stamp

    dyn_ori_pose.pose.position.x = pose_with_cov.pose.pose.position.x
    dyn_ori_pose.pose.position.y = pose_with_cov.pose.pose.position.y
    dyn_ori_pose.pose.position.z = pose_with_cov.pose.pose.position.z

    ori = dyn_ori_pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(r,p,y + ori_deviation*cos( 10 * angle ), axes)

    publisher_cov.publish( pose_with_cov )
    publisher_dyn_pos.publish( dyn_pos_pose )
    publisher_dyn_ori.publish( dyn_ori_pose )

    pos, ori = pose_with_cov.pose.pose.position, pose_with_cov.pose.pose.orientation
    br.sendTransform((pos.x, pos.y, pos.z),
                     (ori.x, ori.y, ori.z, ori.w),
                     stamp,
                     "pose",
                     "base_link")

    angle += .0005
    rate.sleep()

