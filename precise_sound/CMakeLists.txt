cmake_minimum_required(VERSION 2.8.3)
project(precise_sound)

find_package(catkin REQUIRED COMPONENTS rospy actionlib actionlib_msgs)

add_action_files(
    DIRECTORY action
    FILES PlaySounds.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs 
)

# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy actionlib actionlib_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
