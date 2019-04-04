cmake_minimum_required(VERSION 2.8.3)
project(worlds)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

install(DIRECTORY launch Media worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
