/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __MAGNETOMETER_ROS_PLUGIN_HH__
#define __MAGNETOMETER_ROS_PLUGIN_HH__

#include "SensorMagnetic.pb.h"
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <sensor_msgs/MagneticField.h>

namespace gazebo
{
  struct MagnetometerParameters
  {
    /// \brief Intensity of reference earth magnetic field [muT].
    double intensity;
    /// \brief Heading angle of reference earth magnetic field [rad].
    double heading;
    /// \brief Declination of reference earth magnetic field [rad].
    double declination;
    /// \brief Inclination of reference earth magnetic field [rad].
    double inclination;
    /// \brief Discrete-time standard dev. of output noise in xy-axis [muT].
    double noiseXY;
    /// \brief Discrete-time standard dev. of output noise in z-axis [muT].
    double noiseZ;
    /// \brief Standard deviation of constant systematic offset of
    /// measurements [muT].
    double turnOnBias;
  };

  class MagnetometerROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: MagnetometerROSPlugin();

    /// \brief Class destructor
    public: virtual ~MagnetometerROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Magnetometer configuration parameters:
    protected: MagnetometerParameters parameters;

    /// \brief Reference magnetic field in world frame:
    protected: ignition::math::Vector3d magneticFieldWorld;

    /// \brief Constant turn-on bias [muT].
    protected: ignition::math::Vector3d turnOnBias;

    /// \brief Last measurement of magnetic field
    protected: ignition::math::Vector3d measMagneticField;

    /// \brief ROS message
    protected: sensor_msgs::MagneticField rosMsg;
  };
}

#endif // __MAGNETOMETER_ROS_PLUGIN_HH__
