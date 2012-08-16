cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})


add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/covariance_display.cpp
  src/covariance_visual.cpp
  src/odometry_display.cpp
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
