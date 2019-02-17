cmake_minimum_required(VERSION 2.8.3)
project(asv_sim_gazebo_plugins)

###############################################################################
# Compile as C++11, supported in ROS Kinetic and newer

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############################################################################
# Other dependencies...

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
find_package(gazebo REQUIRED)

############################################################################### 
# Catkin...

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    AnemometerSensorPlugin
  DEPENDS
    gazebo_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-MATHS_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-MATHS_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )

add_subdirectory(msgs)

############################################################################### 
# Libraries...

############################################################################### 
# Plugins...

# RegisterSensorsPlugin
add_library(RegisterSensorsPlugin
  SHARED
    src/AnemometerSensor.cc
    src/RegisterSensorsPlugin.cc
)

target_link_libraries(RegisterSensorsPlugin
  asv_sim_gazebo_plugin_msgs
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_compile_options(RegisterSensorsPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_SIM_GAZEBO_PLUGINS_LIST RegisterSensorsPlugin)

# SailLiftDragPlugin
add_library(SailLiftDragPlugin
  SHARED
    src/SailLiftDragPlugin.cc
)

target_link_libraries(SailLiftDragPlugin
  asv_sim_gazebo_plugin_msgs
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_compile_options(SailLiftDragPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_SIM_GAZEBO_PLUGINS_LIST SailLiftDragPlugin)

############################################################################### 
# Executables...

add_executable(SailLiftDragMsgSubscriber
  src/SailLiftDragMsgSubscriber.cc
)

target_link_libraries(SailLiftDragMsgSubscriber  
  asv_sim_gazebo_plugin_msgs
  ${GAZEBO_LIBRARIES}
)

target_compile_options(SailLiftDragMsgSubscriber PRIVATE "-Wno-unknown-pragmas")

add_dependencies(SailLiftDragMsgSubscriber
  asv_sim_gazebo_plugin_msgs
)

list(APPEND ASV_SIM_GAZEBO_EXECUTABLES_LIST SailLiftDragMsgSubscriber)

############################################################################### 
# Tests...
# 
# Useful notes on running catkin tests:
# http://www.personalrobotics.ri.cmu.edu/software/unit-testing
# 

if(CATKIN_ENABLE_TESTING)

endif()

############################################################################### 
# Install

install(TARGETS ${ASV_SIM_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${ASV_SIM_GAZEBO_EXECUTABLES_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)

