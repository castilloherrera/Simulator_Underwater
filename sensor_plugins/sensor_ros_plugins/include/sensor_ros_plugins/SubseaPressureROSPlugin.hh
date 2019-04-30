/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __SUBSEA_PRESSURE_ROS_PLUGIN_HH__
#define __SUBSEA_PRESSURE_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include "SensorPressure.pb.h"
#include <sensor_msgs/FluidPressure.h>

namespace gazebo
{
  class SubseaPressureROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: SubseaPressureROSPlugin();

    /// \brief Class destructor
    public: ~SubseaPressureROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Sensor saturation (max. value for output pressure in Pa)
    protected: double saturation;

    /// \brief If flag is set to true, estimate depth according to pressure
    /// measurement
    protected: bool estimateDepth;

    /// \brief Standard pressure
    protected: double standardPressure;

    /// \brief Factor of kPa per meter
    protected: double kPaPerM;
  };
}

#endif // __SUBSEA_PRESSURE_ROS_PLUGIN_HH__
