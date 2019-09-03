cmake_minimum_required(VERSION 2.8.3)
project(control_msgs)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  geometry_msgs 
  message_generation)

add_message_files(
  FILES
  Trajectory.msg
  TrajectoryPoint.msg
  Waypoint.msg
  WaypointSet.msg
 )

 ## Generate services in the 'srv' folder
 add_service_files(
   FILES
   AddWaypoint.srv
   ClearWaypoints.srv
   InitWaypointsFromFile.srv
   GetWaypoints.srv
   GoTo.srv
   GoToIncremental.srv
   Hold.srv
   IsRunningTrajectory.srv
   InitWaypointSet.srv
   InitRectTrajectory.srv
   InitCircularTrajectory.srv
   InitHelicalTrajectory.srv
   StartTrajectory.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs 
    geometry_msgs 
    message_runtime
)
