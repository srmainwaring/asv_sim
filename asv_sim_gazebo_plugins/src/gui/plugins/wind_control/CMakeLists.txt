if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)

find_package(gz-sim7 REQUIRED COMPONENTS gui)

QT5_ADD_RESOURCES(resources_RCC WindControl.qrc)

add_library(WindControl SHARED
  WindControl.cc
  ${resources_RCC}
)
target_link_libraries(WindControl
  PRIVATE gz-sim7::gui
)

install(TARGETS WindControl DESTINATION ${GZ_SIM_GUI_PLUGIN_INSTALL_DIR})
