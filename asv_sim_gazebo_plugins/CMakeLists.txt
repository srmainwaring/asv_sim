cmake_minimum_required(VERSION 3.10.2)

#============================================================================
# Initialize the project
#============================================================================
project(asv_sim VERSION 2.0.0)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project()

#============================================================================
# Include cmake
#============================================================================

#============================================================================
# Set project-specific options
#============================================================================

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find gz-math
gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
gz_find_package(gz-common5
  REQUIRED COMPONENTS events
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find gz-sim
gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

#--------------------------------------
# Find SDFormat
gz_find_package(sdformat13 REQUIRED)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

#============================================================================

# Location of "fake install folder" used in tests
# Defined here at root scope so it is available for tests in src and test folders
set(FAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/fake/install")

#============================================================================
# Configure the build
#============================================================================

gz_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# Create package information
#============================================================================
gz_create_packages()

# TODO(srmainwaring) - remove ROS / gazebo11 build below when migrated
###############################################################################
# Other dependencies...

# find_package(Boost REQUIRED)
# find_package(Protobuf REQUIRED)

############################################################################### 
# Messages...

# add_subdirectory(msgs)

############################################################################### 
# Libraries...

# asv_sim
# add_library(asv_sim
#   SHARED
#     src/AnemometerSensor.cc
#     src/LiftDragModel.cc
#     src/Utilities.cc
# )

# target_link_libraries(asv_sim
#   asv_sim_msgs
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# add_dependencies(asv_sim
#   ${catkin_EXPORTED_TARGETS}
# )

############################################################################### 
# Plugins...

# RegisterSensorsPlugin
# add_library(RegisterSensorsPlugin
#   SHARED
#     src/RegisterSensorsPlugin.cc
# )

# target_link_libraries(RegisterSensorsPlugin
#   asv_sim
#   asv_sim_msgs
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# add_dependencies(RegisterSensorsPlugin
#   ${catkin_EXPORTED_TARGETS}
# )

# SailPlugin
# add_library(SailPlugin
#   SHARED
#     src/SailPlugin.cc
# )

# target_link_libraries(SailPlugin
#   asv_sim
#   asv_sim_msgs
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# add_dependencies(SailPlugin
#   ${catkin_EXPORTED_TARGETS}
# )

# FoilPlugin
# add_library(FoilPlugin
#   SHARED
#     src/FoilPlugin.cc
# )

# target_link_libraries(FoilPlugin
#   asv_sim
#   asv_sim_msgs
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# add_dependencies(FoilPlugin
#   ${catkin_EXPORTED_TARGETS}
# )

# gazebo_ros_anemometer
# add_library(gazebo_ros_anemometer
#   SHARED
#     src/GazeboRosAnemometer.cc
# )

# target_link_libraries(gazebo_ros_anemometer
#   asv_sim
#   asv_sim_msgs
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# add_dependencies(FoilPlugin
#   ${catkin_EXPORTED_TARGETS}
# )

############################################################################### 
# Executables...

# add_executable(LiftDragMsgSubscriber
#   src/LiftDragMsgSubscriber.cc
# )

# target_link_libraries(LiftDragMsgSubscriber  
#   asv_sim_msgs
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# add_dependencies(LiftDragMsgSubscriber
#   ${catkin_EXPORTED_TARGETS}
# )
