find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_PROTO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set (msgs
  lift_drag.proto
#   anemometer.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(asv_sim_gazebo_plugins_msgs SHARED ${PROTO_SRCS})

target_link_libraries(asv_sim_gazebo_plugins_msgs
  ${PROTOBUF_LIBRARY}
  ${GAZEBO_LIBRARIES}
  ${GAZEBO_PROTO_LIBRARIES}
)

############################################################################### 
# Install

install(TARGETS asv_sim_gazebo_plugins_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
