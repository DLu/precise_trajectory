cmake_minimum_required(VERSION 2.8.3)
project(graph_trajectory)

find_package(catkin REQUIRED COMPONENTS rospy pr2_precise_trajectory)

catkin_python_setup()

catkin_package(
    DEPENDS rospy pr2_precise_trajectory
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)