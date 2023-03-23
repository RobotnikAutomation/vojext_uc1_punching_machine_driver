cmake_minimum_required(VERSION 2.8.3)
project(punching_machine_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  rcomponent
  robotnik_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    punching_data.msg
)

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES
    punching_machine_data.srv
    get_modbus_coil.srv
    set_modbus_coil.srv
)

## Generate added messages and services with any dependencies listed here

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    rcomponent
    message_runtime
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/punching_machine_driver/punching_driver.py
  src/punching_machine_driver/punching_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Modify this if launch or config folders are deleted
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
