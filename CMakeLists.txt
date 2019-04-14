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

add_executable(devices_node src/devices_node.cpp)
target_link_libraries(devices_node ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS devices_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
