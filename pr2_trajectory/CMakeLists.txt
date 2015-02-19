cmake_minimum_required(VERSION 2.8.3)
project(pr2_trajectory)

find_package(catkin REQUIRED COMPONENTS rospy pr2_controllers_msgs actionlib trajectory_msgs trajectory_tools sensor_msgs arm_navigation_msgs joint_limit_tools control_msgs mocap_analysis)

catkin_package(
    DEPENDS rospy pr2_controllers_msgs actionlib trajectory_msgs trajectory_tools sensor_msgs arm_navigation_msgs joint_limit_tools control_msgs mocap_analysis
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)