cmake_minimum_required(VERSION 2.8.3)
project(precise_sound)

find_package(catkin REQUIRED COMPONENTS rospy actionlib)

add_message_files(
  FILES
)
generate_messages(
)

# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy actionlib
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)