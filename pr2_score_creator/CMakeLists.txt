cmake_minimum_required(VERSION 2.8.3)
project(pr2_score_creator)

find_package(catkin REQUIRED COMPONENTS rospy joy_listener sensor_msgs pr2_precise_trajectory pr2_mechanism_msgs graph_trajectory)

catkin_package(
    DEPENDS rospy joy_listener sensor_msgs pr2_precise_trajectory pr2_mechanism_msgs graph_trajectory
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)