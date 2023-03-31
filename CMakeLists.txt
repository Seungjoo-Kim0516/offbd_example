cmake_minimum_required(VERSION 3.0.2)
project(offbd_cpp)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    mavros
    mavros_msgs
    std_msgs
    tf
    geometry_msgs
)

generate_messages(
    DEPENDENCIES
    mavros_msgs   std_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ftm_planning
  CATKIN_DEPENDS mavros mavros_msgs roscpp std_msgs rospy geometry_msgs
#  DEPENDS system_lib
)


include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
    ~/catkin_ws/devel/include
)

add_executable(offbd_node offbd_node.cpp)

    target_link_libraries(offbd_node
    ${catkin_LIBRARIES}
)

add_dependencies(offbd_node ${offbd_cpp_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


