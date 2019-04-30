/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__
#define __CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <sensor_ros_plugins_msgs/ChemicalParticleConcentration.h>
#include <sensor_ros_plugins_msgs/Salinity.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_ros_plugins/ROSBaseModelPlugin.hh>

namespace gazebo
{
  class CPCROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: CPCROSPlugin();

    /// \brief Class destructor
    public: virtual ~CPCROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Update callback from simulator.
    protected: virtual void OnPlumeParticlesUpdate(
      const sensor_msgs::PointCloud::ConstPtr &_msg);

    /// \brief Input topic for the plume particle point cloud
    protected: ros::Subscriber particlesSub;

    /// \brief Output topic for salinity measurements based on the particle concentration
    protected: ros::Publisher salinityPub;

    /// \brief Flag to ensure the cloud and measurement update don't coincide
    protected: bool updatingCloud;

    /// \brief Gamma velocity parameter for the smoothing function
    protected: double gamma;

    /// \brief Sensor gain
    protected: double gain;

    // \brief Radius of the kernel to identify particles that will be taken into
    // account in the concentration computation
    protected: double smoothingLength;

    /// \brief Last update from the point cloud callback
    protected: ros::Time lastUpdateTimestamp;

    /// \brief Output measurement topic
    protected: sensor_ros_plugins_msgs::ChemicalParticleConcentration
      outputMsg;

    /// \brief Output salinity measurement message
    protected: sensor_ros_plugins_msgs::Salinity salinityMsg;

    protected: double waterSalinityValue;

    protected: double plumeSalinityValue;
  };
}

#endif // __CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__
