cmake_minimum_required(VERSION 2.8.3)
project(utils)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS src/trajectory_marker_publisher.py
                               src/send_waypoint_file.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
