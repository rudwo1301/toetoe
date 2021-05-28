cmake_minimum_required(VERSION 3.0.2)
project(toetoe_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    mujoco_ros_msgs
    roscpp
    roslib
    sensor_msgs
    std_msgs
    eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    DEPENDS  geometry_msgs roscpp sensor_msgs std_msgs eigen_conversions mujoco_ros_msgs
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE Headerfiles "include/*.h")
add_custom_target(toetoe_control_headers SOURCES ${HeaderFiles})

add_executable(toetoe_controller src/toetoe_controller.cpp)

target_link_libraries(toetoe_controller
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}

    -L/usr/local/lib
    -lrbdl
    -lrbdl_urdfreader

    -lqpOASES
)

add_dependencies(toetoe_controller ${catkin_EXPORTED_TARGETS})
