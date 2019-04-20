cmake_minimum_required(VERSION 2.8.3)
project(ros_node_configuration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    diagnostic_updater
    serial
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_devices
  CATKIN_DEPENDS
    roscpp
    std_msgs
    diagnostic_updater
    serial
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_devices
  src/device/serial_device.cpp
)
target_link_libraries(${PROJECT_NAME}_devices ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_devices_node src/devices_node.cpp)
add_dependencies(${PROJECT_NAME}_devices_node ${PROJECT_NAME}_devices)
target_link_libraries(${PROJECT_NAME}_devices_node ${PROJECT_NAME}_devices ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_devices_node PROPERTIES OUTPUT_NAME devices_node PREFIX "")

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_devices
    ${PROJECT_NAME}_devices_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config include/${PROJECT_NAME} launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
