/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __ROS_BASE_SENSOR_PLUGIN_HH__
#define __ROS_BASE_SENSOR_PLUGIN_HH__

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_ros_plugins/ROSBasePlugin.hh>
#include <boost/bind.hpp>
#include <string>

namespace gazebo
{
  class ROSBaseSensorPlugin : public ROSBasePlugin, public SensorPlugin
  {
    /// \brief Class constructor
    public: ROSBaseSensorPlugin();

    /// \brief Class destructor
    public: virtual ~ROSBaseSensorPlugin();

    /// \brief Load plugin and its configuration from sdf,
    protected: virtual void Load(sensors::SensorPtr _model,
      sdf::ElementPtr _sdf);

    /// \brief Update callback from simulation.
    protected: virtual bool OnUpdate(const common::UpdateInfo&);

    /// \brief Pointer to the parent sensor
    protected: sensors::SensorPtr parentSensor;
  };
}

#endif // __ROS_BASE_SENSOR_PLUGIN_HH__
