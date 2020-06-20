cmake_minimum_required(VERSION 2.8.3)
project(bbot_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
