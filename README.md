rviz plugin to display pose covariance
===========================================================

This repository provides rviz plugin to display pose covariance information using an ellipsoid and a cone.

The plugin currently supports `geometry_msgs/PoseWithCovariance`, `geometry_msgs/PoseWithCovarianceStamped` and `nav_msgs/Odometry`. This work is based on [this discussion] [discussion] and on this [tutorial] [tutorial].

**This package requires at least ROS Fuerte**

Getting started with rviz_plugin_covariance:

**ROS Fuerte**

(instructions here)

**ROS Hydro**

*To build this plugin:*

Checkout the hydro branch into a catkin workspace and run catkin_make

*To test this plugin:*

`roscore`
`rosrun rviz rviz`
`rosrun rviz_plugin_covariance send_test_msgs.py`
Add new rviz display of type `PoseWithCovariance`
Select the `/test_covariance` topic

TO DO: 
- [ ] write instructions for ROS Fuerte
- [ ] check if ROS Groovy is supported
- [ ] migrate `nav_msgs/Odometry` display to ROS Hydro
- [ ] add code improvements seen in ROS Hydro to older versions

[discussion]: http://geus.wordpress.com/2011/09/15/how-to-represent-a-3d-normal-function-with-ros-rviz/
[tutorial]: http://www.ros.org/wiki/rviz/Tutorials/Plugins%3A%20New%20Display%20Type
