cmake_minimum_required(VERSION 2.8.3)
project(worlds_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	gazebo_dev
	gazebo_msgs)
find_package(Boost REQUIRED)
find_package(Protobuf REQUIRED)
#find_package(gazebo_ros REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES underwater_current_plugin
	CATKIN_DEPENDS
		gazebo_msgs
		gazebo_dev
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS})

link_directories(
	${CMAKE_CURRENT_BINARY_DIR}/msgs
        ${GAZEBO_LIBRARY_DIRS})

###########
## Build ##
###########

add_library(underwater_current_plugin
  src/UnderwaterCurrentPlugin.cc
  src/GaussMarkovProcess.cc)
target_link_libraries(underwater_current_plugin
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
		underwater_current_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)


