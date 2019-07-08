cmake_minimum_required(VERSION 2.8.3)
project(second_project)

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
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(CATKIN_DEPENDS)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## ODOMETRY NODE
add_executable(odometry src/odometry_node.cpp
        src/odometry.cpp)
add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})

## IMU NODE
add_executable(imu_transform src/imu/imu_transform_node.cpp
        src/imu/imutransform.cpp)
add_dependencies(imu_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_transform ${catkin_LIBRARIES})

## GPS NODE
add_executable(gps_transform src/gps/gps_node.cpp
        src/gps/gpstransform.cpp)
add_dependencies(gps_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_transform ${catkin_LIBRARIES})