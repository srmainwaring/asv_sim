cmake_minimum_required(VERSION 3.10.2)

#============================================================================
# Initialize the project
#============================================================================
project(asv_sim2 VERSION 2.0.0)

#============================================================================
# Find gz-cmake
#============================================================================
if("$ENV{GZ_VERSION}" STREQUAL "garden"
    OR "$ENV{GZ_VERSION}" STREQUAL "harmonic"
    OR NOT DEFINED "ENV{GZ_VERSION}")
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})
elseif("$ENV{GZ_VERSION}" STREQUAL "ionic")
  find_package(gz-cmake4 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake4_VERSION_MAJOR})
endif()

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

# Garden (default)
if("$ENV{GZ_VERSION}" STREQUAL "garden" OR NOT DEFINED "ENV{GZ_VERSION}")

  #--------------------------------------
  # Find gz-math
  gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
  set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-plugin
  gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-common
  gz_find_package(gz-common5
    REQUIRED COMPONENTS graphics events
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
  # Find gz-sensors
  gz_find_package(gz-sensors7 REQUIRED)
  set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-sim
  gz_find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

  #--------------------------------------
  # Find SDFormat
  gz_find_package(sdformat13 REQUIRED)
  set(SDF_VER ${sdformat13_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Garden")

  # Harmonic
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")

  #--------------------------------------
  # Find gz-math
  gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
  set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-plugin
  gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-common
  gz_find_package(gz-common5
    REQUIRED COMPONENTS graphics events
  )
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-msgs
  gz_find_package(gz-msgs10 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-transport
  gz_find_package(gz-transport13 REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-sensors
  gz_find_package(gz-sensors8 REQUIRED)
  set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-sim
  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  #--------------------------------------
  # Find SDFormat
  gz_find_package(sdformat14 REQUIRED)
  set(SDF_VER ${sdformat14_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")

# Ionic
elseif("$ENV{GZ_VERSION}" STREQUAL "ionic")

  #--------------------------------------
  # Find gz-math
  gz_find_package(gz-math8 REQUIRED COMPONENTS eigen3)
  set(GZ_MATH_VER ${gz-math8_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-plugin
  gz_find_package(gz-plugin3 REQUIRED COMPONENTS loader register)
  set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-common
  gz_find_package(gz-common6
    REQUIRED COMPONENTS graphics events
  )
  set(GZ_COMMON_VER ${gz-common6_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-msgs
  gz_find_package(gz-msgs11 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs11_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-transport
  gz_find_package(gz-transport14 REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-sensors
  gz_find_package(gz-sensors9 REQUIRED)
  set(GZ_SENSORS_VER ${gz-sensors9_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-sim
  gz_find_package(gz-sim9 REQUIRED)
  set(GZ_SIM_VER ${gz-sim9_VERSION_MAJOR})

  #--------------------------------------
  # Find SDFormat
  gz_find_package(sdformat15 REQUIRED)
  set(SDF_VER ${sdformat15_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Ionic")

else()
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

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

