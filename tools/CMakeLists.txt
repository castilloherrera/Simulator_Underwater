cmake_minimum_required(VERSION 2.8.3)
project(tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  topic_tools)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    topic_tools
)

include_directories(${catkin_INCLUDE_DIRS})


catkin_install_python(PROGRAMS scripts/set_simulation_timer.py
                               scripts/publish_world_models.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

