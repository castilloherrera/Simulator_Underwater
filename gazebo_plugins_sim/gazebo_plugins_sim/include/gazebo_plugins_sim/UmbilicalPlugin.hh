/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/
/// \file UmbilicalPlugin.hh
/// \brief Model plugin for the umbilical (tether) of an ROV.

#ifndef __GAZEBO_PLUGINS_SIM_UMBILICAL_PLUGIN_HH__
#define __GAZEBO_PLUGINS_SIM_UMBILICAL_PLUGIN_HH__

#include <memory>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <gazebo_plugins_sim/UmbilicalModel.hh>

namespace gazebo
{
class UmbilicalSegment
{
  public:
    UmbilicalSegment() { initSdfSegment(); }

    UmbilicalSegment(const std::string& _name,
                     const std::string& _fromLink,
                     const ignition::math::Pose3d& _fromPose,
                     const ignition::math::Pose3d& _toPose,
                     physics::ModelPtr _model);

    void initSdfSegment();

    physics::LinkPtr link;
    physics::LinkPtr linkA;
    physics::JointPtr jointA;
    physics::JointPtr jointB;

    std::shared_ptr<UmbilicalSegment> prev, next;

    static sdf::SDFPtr sdfSegment;
};

typedef boost::shared_ptr<UmbilicalSegment> UmbilicalSegmentPtr;

class UmbilicalPlugin : public ModelPlugin
{
  /// \brief Destructor.
  public: UmbilicalPlugin();

  /// \brief Constructor.
  public: ~UmbilicalPlugin();

  /// \brief Load plugin and its configuration from sdf
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Update callback from simulation.
  protected: virtual void OnUpdate(const common::UpdateInfo&);

  /// \brief Reads flow velocity topic
  protected: void UpdateFlowVelocity(ConstVector3dPtr &_msg);

  /// \brief Pointer to the update event connection.
  protected: event::ConnectionPtr updateConnection;

  /// \brief Pointer to the model structure
  protected: gazebo::physics::ModelPtr model;

  /// \brief Pointer to the world plugin
  protected: gazebo::physics::WorldPtr world;

  /// \brief Gazebo node
  protected: gazebo::transport::NodePtr node;

  /// \brief Subcriber to flow message
  protected: gazebo::transport::SubscriberPtr flowSubscriber;

  /// \brief Flow velocity vector read from topic
  protected: ignition::math::Vector3d flowVelocity;

  /// \brief Pointer to UmbilicalModel used in this plugin.
  protected: std::shared_ptr<UmbilicalModel> umbilical;
};
}

#endif
