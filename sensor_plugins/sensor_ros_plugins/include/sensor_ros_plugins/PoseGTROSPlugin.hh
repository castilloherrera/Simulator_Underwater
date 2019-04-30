/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __POSE_GT_SENSOR_ROS_PLUGIN_HH__
#define __POSE_GT_SENSOR_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <sensor_ros_plugins/ROSBaseModelPlugin.hh>

namespace gazebo
{
  class PoseGTROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: PoseGTROSPlugin();

    /// \brief Class destructor
    public: ~PoseGTROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    protected: void PublishNEDOdomMessage(common::Time _time,
      ignition::math::Pose3d _pose, ignition::math::Vector3d _linVel,
      ignition::math::Vector3d _angVel);

    protected: void PublishOdomMessage(common::Time _time,
      ignition::math::Pose3d _pose, ignition::math::Vector3d _linVel,
      ignition::math::Vector3d _angVel);

    protected: void UpdateNEDTransform();

    protected: ros::Publisher nedOdomPub;

    /// \brief Pose offset
    protected: ignition::math::Pose3d offset;

    protected: std::string nedFrameID;

    protected: ignition::math::Pose3d nedTransform;

    protected: bool nedTransformIsInit;

    protected: bool publishNEDOdom;

    protected: tf2_ros::Buffer tfBuffer;

    protected: boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    protected: ignition::math::Vector3d lastLinVel;
    protected: ignition::math::Vector3d lastAngVel;
    protected: ignition::math::Vector3d linAcc;
    protected: ignition::math::Vector3d angAcc;
    protected: ignition::math::Vector3d lastRefLinVel;
    protected: ignition::math::Vector3d lastRefAngVel;
    protected: ignition::math::Vector3d refLinAcc;
    protected: ignition::math::Vector3d refAngAcc;
  };
}

#endif // __POSE_GT_SENSOR_ROS_PLUGIN_HH__
