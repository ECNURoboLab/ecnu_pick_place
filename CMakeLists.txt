cmake_minimum_required(VERSION 2.8.3)
project(ecnu_pick_place)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        filtered_cloud_publisher
        moveit_msgs
        sensor_msgs
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        actionlib
        tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



include_directories(
        include
        /usr/include/eigen3
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
set(SOURCE_FILES src/ecnu_pick_place_node.cpp src/ECNUPickPlace.cpp)

add_library(planning_helper src/PlanningHelper.cpp)
target_link_libraries(planning_helper ${catkin_INCLUDE_DIRS})

add_executable(ecnu_pick_place ${SOURCE_FILES})
target_link_libraries(ecnu_pick_place planning_helper ${catkin_LIBRARIES})