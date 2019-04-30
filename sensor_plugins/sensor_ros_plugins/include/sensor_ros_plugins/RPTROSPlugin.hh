/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __RPT_ROS_PLUGIN_HH__
#define __RPT_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <sensor_ros_plugins_msgs/PositionWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "SensorRpt.pb.h"

namespace gazebo
{
  class RPTROSPlugin :  public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: RPTROSPlugin();

    /// \brief Class destructor
    public: virtual ~RPTROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Latest measured position.
    protected: ignition::math::Vector3d position;

    /// \brief Store message since many attributes do not change (cov.).
    protected: sensor_ros_plugins_msgs::PositionWithCovarianceStamped rosMessage;
  };
}

#endif // __RPT_ROS_PLUGIN_HH__
