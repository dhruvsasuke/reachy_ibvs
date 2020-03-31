cmake_minimum_required(VERSION 2.8.3)
project(reachy_vel_moveit)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy moveit_ros_planning moveit_core message_generation)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(jacobian src/jacobian.cpp)
target_link_libraries(jacobian ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
