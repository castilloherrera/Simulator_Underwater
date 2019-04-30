/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __THRUSTER_ROS_PLUGIN_HH__
#define __THRUSTER_ROS_PLUGIN_HH__

#include <map>
#include <string>
#include <vector>

#include <gazebo_plugins_sim/ThrusterPlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo_ros_plugins_msgs/FloatStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <gazebo_ros_plugins_msgs/SetThrusterState.h>
#include <gazebo_ros_plugins_msgs/GetThrusterState.h>
#include <gazebo_ros_plugins_msgs/SetThrusterEfficiency.h>
#include <gazebo_ros_plugins_msgs/GetThrusterEfficiency.h>
#include <gazebo_ros_plugins_msgs/GetThrusterConversionFcn.h>

namespace uuv_simulator_ros
{
  class ThrusterROSPlugin : public gazebo::ThrusterPlugin
  {
    /// \brief Constrcutor.
    public: ThrusterROSPlugin();

    /// \brief Destructor.
    public: ~ThrusterROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Publish thruster state via ROS.
    public: void RosPublishStates();

    /// \brief Set new set point (desired thrust [N]) for thruster.
    public: void SetThrustReference(
        const gazebo_ros_plugins_msgs::FloatStamped::ConstPtr &_msg);

    /// \brief Return the ROS publish period.
    public: gazebo::common::Time  GetRosPublishPeriod();

    /// \brief Set the ROS publish frequency (Hz).
    public: void SetRosPublishRate(double _hz);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Set the thrust efficiency factor
    public: bool SetThrustForceEfficiency(
      gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request& _req,
      gazebo_ros_plugins_msgs::SetThrusterEfficiency::Response& _res);

    /// \brief Get the thrust efficiency factor
    public: bool GetThrustForceEfficiency(
      gazebo_ros_plugins_msgs::GetThrusterEfficiency::Request& _req,
      gazebo_ros_plugins_msgs::GetThrusterEfficiency::Response& _res);

    /// \brief Set the dynamic state efficiency factor
    public: bool SetDynamicStateEfficiency(
      gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request& _req,
      gazebo_ros_plugins_msgs::SetThrusterEfficiency::Response& _res);

      /// \brief Get the dynamic state efficiency factor
    public: bool GetDynamicStateEfficiency(
        gazebo_ros_plugins_msgs::GetThrusterEfficiency::Request& _req,
        gazebo_ros_plugins_msgs::GetThrusterEfficiency::Response& _res);

    /// \brief Turn thruster on/off
    public: bool SetThrusterState(
      gazebo_ros_plugins_msgs::SetThrusterState::Request& _req,
      gazebo_ros_plugins_msgs::SetThrusterState::Response& _res);

    /// \brief Get thruster state
    public: bool GetThrusterState(
      gazebo_ros_plugins_msgs::GetThrusterState::Request& _req,
      gazebo_ros_plugins_msgs::GetThrusterState::Response& _res);

    /// \brief Get thruster conversion function parameters
    public: bool GetThrusterConversionFcn(
      gazebo_ros_plugins_msgs::GetThrusterConversionFcn::Request& _req,
      gazebo_ros_plugins_msgs::GetThrusterConversionFcn::Response& _res);

    /// \brief Map of thruster services
    private: std::map<std::string, ros::ServiceServer> services;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief Subscriber reacting to new reference thrust set points.
    private: ros::Subscriber subThrustReference;

    /// \brief Publisher for current actual thrust.
    private: ros::Publisher pubThrust;

    /// \brief Publisher for current actual thrust as wrench.
    private: ros::Publisher pubThrustWrench;

    /// \brief Publisher for the thruster state
    private: ros::Publisher pubThrusterState;

    /// \brief Publisher for the thrust force efficiency
    private: ros::Publisher pubThrustForceEff;

    /// \brief Publisher for the dynamic state efficiency
    private: ros::Publisher pubDynamicStateEff;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // __THRUSTER_ROS_PLUGIN_HH__
