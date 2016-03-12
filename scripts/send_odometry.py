#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
from nav_msgs.msg import Odometry
import rospy
import tf
from numpy import pi, cos, sin

br = tf.TransformBroadcaster()

publisher = rospy.Publisher('test_odometry', Odometry, queue_size=5)
publisher_2D = rospy.Publisher('test_odometry_2D', Odometry, queue_size=5)

rospy.init_node('send_odometry')

y = 0
angle = 0

roll = 0
pitch = 0
yaw = pi/2
axes = 'sxyz' # 'sxyz' or 'rxyz'

ori_deviation = pi/6.0;
ori_deviation2 = pi/18.0;

while not rospy.is_shutdown():

   odo = Odometry()
   odo.header.frame_id = "/base_link"
   odo.child_frame_id = "/odom_msg"
   odo.header.stamp = rospy.Time.now()
   
   odo.pose.pose.position.x = 0
   odo.pose.pose.position.y = y
   odo.pose.pose.position.z = 0

   ori = odo.pose.pose.orientation
   ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(roll, pitch, yaw + angle, axes)

   odo.pose.covariance[0+0*6] = 0.2+(abs(y)/2.5);
   odo.pose.covariance[1+1*6] = 0.2;
   odo.pose.covariance[2+2*6] = 0.2;
   odo.pose.covariance[3+3*6] = ori_deviation2**2.0
   odo.pose.covariance[4+4*6] = ori_deviation2**2.0
   odo.pose.covariance[5+5*6] = ori_deviation**2.0

   br.sendTransform((odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z),
      (odo.pose.pose.orientation.x,odo.pose.pose.orientation.y,odo.pose.pose.orientation.z,odo.pose.pose.orientation.w),
      odo.header.stamp,
      "odom_msg",
      "base_link")

   publisher.publish( odo )

   odo.pose.covariance[2+2*6] = 0.0;
   odo.pose.covariance[3+3*6] = 0.0;
   odo.pose.covariance[4+4*6] = 0.0;

   publisher_2D.publish( odo )


   y = y + .02
   if y > 5:
      y = -5

   angle += .005
   if angle > 2*pi:
      angle -= 2*pi

   rospy.sleep(0.01)
