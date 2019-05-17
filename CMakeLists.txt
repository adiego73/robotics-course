cmake_minimum_required(VERSION 2.8.3)
project(ros_project_a)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        roscpp
        rospy
        std_msgs
        tf
        custom_messages
        message_filters
        message_generation
        dynamic_reconfigure
)

add_message_files(
        FILES
        floatStamped.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

generate_dynamic_reconfigure_options(
        cfg/config_params.cfg
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(CATKIN_DEPENDS custom_messages message_filters)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(transform src/transform_node.cpp src/odometry_differential.cpp include/odometry_differential.h)
add_dependencies(transform ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(param_second ${PROJECT_NAME}_gencfg)
target_link_libraries(transform ${catkin_LIBRARIES})
