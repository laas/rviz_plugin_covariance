cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_covariance)
find_package(catkin REQUIRED COMPONENTS rviz cmake_modules)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/pose_with_covariance_display.h
  src/odometry_display.h
  src/covariance_visual.h
  src/covariance_property.h
)

set(SOURCE_FILES
  src/covariance_visual.cpp
  src/covariance_property.cpp
  src/pose_with_covariance_display.cpp
  src/odometry_display.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
