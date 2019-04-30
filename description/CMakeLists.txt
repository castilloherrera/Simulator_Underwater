cmake_minimum_required(VERSION 2.8.3)
project(description)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/spawn_model
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


