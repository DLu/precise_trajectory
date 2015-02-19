cmake_minimum_required(VERSION 2.8.3)
project(pr2_pose_morph)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs sensor_msgs tf easy_markers actionlib actionlib_msgs pr2_controllers_msgs)

catkin_package(
    DEPENDS rospy geometry_msgs sensor_msgs tf easy_markers actionlib actionlib_msgs pr2_controllers_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)