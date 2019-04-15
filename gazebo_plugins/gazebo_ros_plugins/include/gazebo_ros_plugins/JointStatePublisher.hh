/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

/// \file JointStatePublisher.hh A Gazebo ROS plugin for publishing the joint
/// states of a robot (position, velocity and effort). Build similar to the
/// class in the GazeboRosJointStatePublisher, but including more information

#ifndef __JOINT_STATE_PUBLISHER_HH__
#define __JOINT_STATE_PUBLISHER_HH__

#include <string>
#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sstream>

namespace uuv_simulator_ros
{
class JointStatePublisher : public gazebo::ModelPlugin
{
  public: JointStatePublisher();

  public: ~JointStatePublisher();

  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  public: void OnUpdate(const gazebo::common::UpdateInfo &_info);

  public: void PublishJointStates();

  private: bool IsIgnoredJoint(std::string _jointName);

  private: gazebo::physics::WorldPtr world;

  private: gazebo::physics::ModelPtr model;

  private: gazebo::event::ConnectionPtr updateConnection;

  private: boost::shared_ptr<ros::NodeHandle> node;

  private: std::string robotNamespace;

  private: std::vector<std::string> movingJoints;

  private: double updateRate;

  private: double updatePeriod;

  private: gazebo::common::Time lastUpdate;

  private: ros::Publisher jointStatePub;
};
}

#endif  // __JOINT_STATE_PUBLISHER_HH__
