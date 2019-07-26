cmake_minimum_required(VERSION 2.8.3)
project(follower)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES Car.msg)

## Generate added messages and services with any dependencies listed here
 generate_messages(DEPENDENCIES std_msgs nav_msgs)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(follower src/follower.cpp src/vector.cpp src/bezier.cpp src/velocity_profile.cpp src/pid.cpp)
target_link_libraries(follower ${catkin_LIBRARIES})
