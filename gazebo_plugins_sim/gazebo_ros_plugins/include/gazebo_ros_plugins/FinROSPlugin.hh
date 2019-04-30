/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __FIN_ROS_PLUGIN_HH__
#define __FIN_ROS_PLUGIN_HH__

#include <gazebo_plugins_sim/FinPlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo_ros_plugins_msgs/FloatStamped.h>
#include <gazebo_ros_plugins_msgs/GetListParam.h>
#include <geometry_msgs/WrenchStamped.h>
#include <map>

namespace uuv_simulator_ros
{
  class FinROSPlugin : public gazebo::FinPlugin
  {
    /// \brief Constrcutor.
    public: FinROSPlugin();

    /// \brief Destructor.
    public: ~FinROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Publish state via ROS.
    public: void RosPublishStates();

    /// \brief Set new set point.
    public: void SetReference(
        const gazebo_ros_plugins_msgs::FloatStamped::ConstPtr &_msg);

    /// \brief Return the list of paramaters of the lift and drag model
    public: bool GetLiftDragParams(
      gazebo_ros_plugins_msgs::GetListParam::Request& _req,
      gazebo_ros_plugins_msgs::GetListParam::Response& _res);

    /// \brief Return the ROS publish period.
    public: gazebo::common::Time GetRosPublishPeriod();

    /// \brief Set the ROS publish frequency (Hz).
    public: void SetRosPublishRate(double _hz);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief Subscriber reacting to new reference set points.
    private: ros::Subscriber subReference;

    /// \brief Publisher for current state.
    private: ros::Publisher pubState;

    /// \brief Publisher for current actual thrust.
    private: ros::Publisher pubFinForce;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Map of services
    private: std::map<std::string, ros::ServiceServer> services;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif
