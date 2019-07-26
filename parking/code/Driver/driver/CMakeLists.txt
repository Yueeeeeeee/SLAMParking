cmake_minimum_required (VERSION 3.7.2)
project(driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-Wall -g -Wfatal-errors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

# add sub directories to CMake build-tree
add_subdirectory(subs/af3pihal)
add_subdirectory(subs/vesc)


## Declare ROS messages and services
add_message_files(FILES Car.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)


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
add_executable(${PROJECT_NAME} src/main.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} af3pihal vesc pthread)
