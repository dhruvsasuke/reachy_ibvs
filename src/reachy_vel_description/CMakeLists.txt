cmake_minimum_required(VERSION 2.8.3)
project(reachy_vel_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy cmake_modules eigen_conversions)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime eigen)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(transform src/transform.cpp)
target_link_libraries(transform ${catkin_LIBRARIES})

