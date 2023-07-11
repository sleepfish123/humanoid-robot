cmake_minimum_required(VERSION 3.0.2)
project(big_grey_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_ros_perception
  actionlib
  moveit_msgs
  roscpp
  rospy
  serial
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  darknet_ros_msgs
  message_generation

  move_base_msgs
  nav_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3.4.7 REQUIRED )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  big_grey_srv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  moveit_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES big_grey_control
  CATKIN_DEPENDS message_runtime actionlib moveit_msgs roscpp rospy serial std_msgs moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/big_grey_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/big_grey_control_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_big_grey_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(servo_command include/big_grey_control/servo_command.h src/servo_command.cpp)
add_dependencies(servo_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(RobotArm include/big_grey_control/RobotArm.h src/RobotArm.cpp)
add_dependencies(RobotArm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(RobotClaw include/big_grey_control/RobotClaw.h src/RobotClaw.cpp)
add_dependencies(RobotClaw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(RobotWaist include/big_grey_control/RobotWaist.h src/RobotWaist.cpp)
add_dependencies(RobotWaist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(RobotCar include/big_grey_control/RobotCar.h src/RobotCar.cpp)
add_dependencies(RobotCar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(serial_control src/serial_control.cpp)
# add_dependencies(serial_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(serial_control servo_command ${catkin_LIBRARIES})



add_executable(waist_init src/waist_init.cpp)
add_dependencies(waist_init ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waist_init RobotWaist ${catkin_LIBRARIES})

add_executable(image_depth_rgb include/big_grey_control/image.h src/image_depth_rgb.cpp)
add_dependencies(image_depth_rgb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_depth_rgb ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(send_apple_site src/send_apple_site.cpp)
add_dependencies(send_apple_site ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(send_apple_site ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_send_apple_site src/test_send_apple_site.cpp)
add_dependencies(test_send_apple_site ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_send_apple_site ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(grasp src/grasp.cpp)
add_dependencies(grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grasp RobotClaw RobotCar RobotWaist servo_command RobotArm ${catkin_LIBRARIES})

add_executable(big_grey_one_button_start src/big_grey_one_button_start.cpp)
add_dependencies(big_grey_one_button_start ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(big_grey_one_button_start ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(manipulator_init src/manipulator_init.cpp)
add_dependencies(manipulator_init ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_init servo_command RobotArm RobotClaw ${catkin_LIBRARIES})

add_executable(robot_test src/robot_test.cpp)
add_dependencies(robot_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_test RobotArm RobotClaw RobotCar RobotWaist ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(robot_waist_test src/robot_waist_test.cpp)
add_dependencies(robot_waist_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_waist_test RobotWaist ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(test_vision src/test_vision.cpp)
add_dependencies(test_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_vision RobotArm RobotClaw ${catkin_LIBRARIES})

add_executable(head_track_control src/head_track_control.cpp)
add_dependencies(head_track_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_track_control  RobotArm  ${catkin_LIBRARIES})

add_executable(vision_head_control src/vision_head_control.cpp)
add_dependencies(vision_head_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_head_control   ${catkin_LIBRARIES})

add_executable(pub_ceshi src/pub_ceshi.cpp)
add_dependencies(pub_ceshi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_ceshi   ${catkin_LIBRARIES})

add_executable(vision_arm_control src/vision_arm_control.cpp)
add_dependencies(vision_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_arm_control   ${catkin_LIBRARIES})

add_executable(arm_track_control src/arm_track_control.cpp)
add_dependencies(arm_track_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_track_control  RobotArm ${catkin_LIBRARIES})





add_library(RobotCarControl include/big_grey_control/RobotCarControl.h src/RobotCarControl.cpp)
add_dependencies(RobotCarControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(big_grey_brain src/big_grey_brain.cpp)
add_dependencies(big_grey_brain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(big_grey_brain RobotArm RobotClaw RobotWaist RobotCarControl ${catkin_LIBRARIES})

add_executable(fabrik_demo src/fabrik_demo.cpp)
add_dependencies(fabrik_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fabrik_demo RobotWaist RobotArm RobotClaw RobotCar RobotCarControl ${catkin_LIBRARIES})

add_executable(car_init src/car_init.cpp)
add_dependencies(car_init ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_init RobotCarControl ${catkin_LIBRARIES})

add_executable(car_test src/car_test.cpp)
add_dependencies(car_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_test RobotCarControl ${catkin_LIBRARIES} ${OpenCV_LIBS})



add_executable(timer_ros src/timer_ros.cpp)
add_dependencies(timer_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(timer_ros RobotWaist RobotCarControl ${catkin_LIBRARIES})

add_executable(movebase_grasp_test src/movebase_grasp_test.cpp)
add_dependencies(movebase_grasp_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(movebase_grasp_test ${catkin_LIBRARIES})

add_executable(main_bigwhite src/main_bigwhite.cpp)
add_dependencies(main_bigwhite ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_bigwhite ${catkin_LIBRARIES})

# 测试机械臂电机
# target_link_libraries(${PROJECT_SOURCE_DIR}/include/big_grey_control)
# add_library(RobotCarControl include/big_grey_control/RobotCarControl.h src/RobotCarControl.cpp)
# add_dependencies(RobotCarControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(motor_public include/big_grey_control/motor_public.h src/motor_public.cpp)
add_dependencies(motor_public ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(motor_motorcommand include/big_grey_control/motor_motorcommand.h src/motor_motorcommand.cpp)
add_dependencies(motor_motorcommand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motor_main src/motor_main.cpp)
add_dependencies(motor_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_main motor_public motor_motorcommand ${catkin_LIBRARIES})

add_executable(arm_test src/arm_test.cpp)
add_dependencies(arm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_test ${catkin_LIBRARIES})