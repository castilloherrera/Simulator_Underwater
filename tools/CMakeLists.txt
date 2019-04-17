cmake_minimum_required(VERSION 2.8.3)
project(tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf
  topic_tools)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    tf
    topic_tools
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(message_to_tf
  src/message_to_tf.cc
)

target_link_libraries(message_to_tf
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS scripts/set_simulation_timer.py
                               scripts/publish_world_models.py
                               scripts/spawn_model
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

install(TARGETS message_to_tf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
