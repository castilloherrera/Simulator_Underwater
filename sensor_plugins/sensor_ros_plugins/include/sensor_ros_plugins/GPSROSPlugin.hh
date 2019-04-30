/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __GPS_SENSOR_ROS_PLUGIN_HH__
#define __GPS_SENSOR_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_ros_plugins/ROSBaseSensorPlugin.hh>
#include <ros/ros.h>

namespace gazebo
{
  class GPSROSPlugin : public ROSBaseSensorPlugin
  {
    /// \brief Class constructor
    public: GPSROSPlugin();

    /// \brief Class destructor
    public: virtual ~GPSROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: virtual void Load(sensors::SensorPtr _parent,
      sdf::ElementPtr _sdf);

    /// \brief Update GPS ROS message
    public: bool OnUpdateGPS();

    /// \brief Pointer to the parent sensor
    protected: sensors::GpsSensorPtr gazeboGPSSensor;

    /// \brief Output GPS ROS message
    protected: sensor_msgs::NavSatFix gpsMessage;
  };
}

#endif // __GPS_SENSOR_ROS_PLUGIN_HH__
