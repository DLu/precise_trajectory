cmake_minimum_required(VERSION 2.8.3)
project(pr2_kinect_teleop)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs pr2_controllers_msgs pr2_gripper_sensor_msgs pr2_precise_trajectory actionlib)

catkin_package(
    DEPENDS rospy sensor_msgs pr2_controllers_msgs pr2_gripper_sensor_msgs pr2_precise_trajectory actionlib
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)