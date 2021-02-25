cmake_minimum_required(VERSION 2.8.3)
project(asv_sim_gazebo_plugins)

###############################################################################
# Compile as C++17, required by Gazebo11

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############################################################################
# Other dependencies...

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  std_msgs
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(Protobuf REQUIRED)
# find_package(GTest REQUIRED)

############################################################################### 
# Catkin...

catkin_package(
    INCLUDE_DIRS
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${GAZEBO_PROTO_INCLUDE_DIRS}
    LIBRARIES
        asv_sim_gazebo_plugins
        asv_sim_gazebo_plugins_msgs
        RegisterSensorsPlugin
        SailLiftDragPlugin
    DEPENDS
        gazebo_ros
        geometry_msgs
        roscpp
        std_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${GAZEBO_PROTO_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-MATHS_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-MATHS_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
  # ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )

############################################################################### 
# Lists

set(ASV_SIM_GAZEBO_LIBRARIES_LIST "")
set(ASV_SIM_GAZEBO_PLUGINS_LIST "")
set(ASV_SIM_GAZEBO_EXECUTABLES_LIST "")

############################################################################### 
# Messages...

add_subdirectory(msgs)

############################################################################### 
# Libraries...

# asv_sim_gazebo_plugins
add_library(asv_sim_gazebo_plugins
  SHARED
    # src/AnemometerSensor.cc
    src/LiftDragModel.cc
    src/Utilities.cc
)

target_link_libraries(asv_sim_gazebo_plugins
  asv_sim_gazebo_plugins_msgs
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(asv_sim_gazebo_plugins
  ${catkin_EXPORTED_TARGETS}
)

target_compile_options(asv_sim_gazebo_plugins PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_SIM_GAZEBO_LIBRARIES_LIST asv_sim_gazebo_plugins)

############################################################################### 
# Plugins...

# RegisterSensorsPlugin
add_library(RegisterSensorsPlugin
  SHARED
    src/RegisterSensorsPlugin.cc
)

target_link_libraries(RegisterSensorsPlugin
  asv_sim_gazebo_plugins
  asv_sim_gazebo_plugins_msgs
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(RegisterSensorsPlugin
  ${catkin_EXPORTED_TARGETS}
)

target_compile_options(RegisterSensorsPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_SIM_GAZEBO_PLUGINS_LIST RegisterSensorsPlugin)

# SailPlugin
add_library(SailPlugin
  SHARED
    src/SailPlugin.cc
)

target_link_libraries(SailPlugin
  asv_sim_gazebo_plugins
  asv_sim_gazebo_plugins_msgs
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(SailPlugin
  ${catkin_EXPORTED_TARGETS}
)

target_compile_options(SailPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_SIM_GAZEBO_PLUGINS_LIST SailPlugin)

# FoilPlugin
add_library(FoilPlugin
  SHARED
    src/FoilPlugin.cc
)

target_link_libraries(FoilPlugin
  asv_sim_gazebo_plugins
  asv_sim_gazebo_plugins_msgs
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(FoilPlugin
  ${catkin_EXPORTED_TARGETS}
)

target_compile_options(FoilPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_SIM_GAZEBO_PLUGINS_LIST FoilPlugin)

# gazebo_ros_anemometer
# add_library(gazebo_ros_anemometer
#   SHARED
#     src/GazeboRosAnemometer.cc
# )

# target_link_libraries(gazebo_ros_anemometer
#   asv_sim_gazebo_plugins
#   asv_sim_gazebo_plugins_msgs
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# add_dependencies(gazebo_ros_anemometer
#   ${catkin_EXPORTED_TARGETS}
# )

# target_compile_options(gazebo_ros_anemometer PRIVATE "-Wno-unknown-pragmas")

# list(APPEND ASV_SIM_GAZEBO_PLUGINS_LIST gazebo_ros_anemometer)

############################################################################### 
# Executables...

add_executable(LiftDragMsgSubscriber
  src/LiftDragMsgSubscriber.cc
)

target_link_libraries(LiftDragMsgSubscriber  
  asv_sim_gazebo_plugins_msgs
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_compile_options(LiftDragMsgSubscriber PRIVATE "-Wno-unknown-pragmas")


add_dependencies(LiftDragMsgSubscriber
  ${catkin_EXPORTED_TARGETS}
)

list(APPEND ASV_SIM_GAZEBO_EXECUTABLES_LIST LiftDragMsgSubscriber)

############################################################################### 
# Tests...
# 
# Useful notes on running catkin tests:
# http://www.personalrobotics.ri.cmu.edu/software/unit-testing
# 

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(UNIT_LiftDragModel_TEST src/LiftDragModel_TEST.cc)
    target_link_libraries(UNIT_LiftDragModel_TEST asv_sim_gazebo_plugins)
    target_compile_options(UNIT_LiftDragModel_TEST PRIVATE "-Wno-unknown-pragmas")  
endif()

############################################################################### 
# Install

install(TARGETS ${ASV_SIM_GAZEBO_LIBRARIES_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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

