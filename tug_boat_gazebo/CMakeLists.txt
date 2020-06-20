cmake_minimum_required(VERSION 2.8.3)
project(tug_boat_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
