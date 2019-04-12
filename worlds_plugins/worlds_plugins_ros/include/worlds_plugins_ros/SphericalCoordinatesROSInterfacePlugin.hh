/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

/// \file SphericalCoordinatesROSInterfacePlugin.hh

#ifndef __SC_ROS_INTERFACE_PLUGIN_HH__
#define __SC_ROS_INTERFACE_PLUGIN_HH__

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>
#include <worlds_plugins_ros_msgs/SetOriginSphericalCoord.h>
#include <worlds_plugins_ros_msgs/GetOriginSphericalCoord.h>
#include <worlds_plugins_ros_msgs/TransformToSphericalCoord.h>
#include <worlds_plugins_ros_msgs/TransformFromSphericalCoord.h>
#include <geometry_msgs/Vector3.h>

#include <map>
#include <string>

namespace gazebo
{

class SphericalCoordinatesROSInterfacePlugin : public WorldPlugin
{
  /// \brief Constructor
  public: SphericalCoordinatesROSInterfacePlugin();

  /// \brief Destructor
  public: virtual ~SphericalCoordinatesROSInterfacePlugin();

  /// \brief Load module and read parameters from SDF.
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief Service call that returns the origin in WGS84 standard
  public: bool GetOriginSphericalCoord(
      worlds_plugins_ros_msgs::GetOriginSphericalCoord::Request& _req,
      worlds_plugins_ros_msgs::GetOriginSphericalCoord::Response& _res);

  /// \brief Service call that returns the origin in WGS84 standard
  public: bool SetOriginSphericalCoord(
      worlds_plugins_ros_msgs::SetOriginSphericalCoord::Request& _req,
      worlds_plugins_ros_msgs::SetOriginSphericalCoord::Response& _res);

  /// \brief Service call to transform from Cartesian to spherical coordinates
  public: bool TransformToSphericalCoord(
      worlds_plugins_ros_msgs::TransformToSphericalCoord::Request& _req,
      worlds_plugins_ros_msgs::TransformToSphericalCoord::Response& _res);

  /// \brief Service call to transform from spherical to Cartesian coordinates
  public: bool TransformFromSphericalCoord(
      worlds_plugins_ros_msgs::TransformFromSphericalCoord::Request& _req,
      worlds_plugins_ros_msgs::TransformFromSphericalCoord::Response& _res);

  /// \brief Pointer to this ROS node's handle.
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief Connection for callbacks on update world.
  protected: event::ConnectionPtr rosPublishConnection;

  /// \brief Pointer to world
  protected: physics::WorldPtr world;

  /// \brief All underwater world services
  protected: std::map<std::string, ros::ServiceServer> worldServices;
};

}

#endif // __SC_ROS_INTERFACE_PLUGIN_HH__
