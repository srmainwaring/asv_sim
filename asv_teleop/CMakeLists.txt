cmake_minimum_required(VERSION 2.8.3)
project(asv_teleop)
find_package(catkin REQUIRED)

############################################################################### 
# Python...
# Uncomment this if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

catkin_python_setup()

############################################################################### 
# Catkin...

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs
)

############################################################################### 
# Install...

catkin_install_python(
  PROGRAMS
    bin/listener
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE
)


