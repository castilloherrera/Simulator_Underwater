/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

/// \file FinPlugin.hh
/// \brief Model plugin for description of a submarine's fin.

#ifndef __GAZEBO_PLUGINS_SIM_FIN_PLUGIN_HH__
#define __GAZEBO_PLUGINS_SIM_FIN_PLUGIN_HH__

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo_plugins_sim/Dynamics.hh>
#include <gazebo_plugins_sim/LiftDragModel.hh>

#include "Double.pb.h"

namespace gazebo {

/// \brief Definition of a pointer to the floating point message
typedef const boost::shared_ptr<const gazebo_plugins_msgs::msgs::Double>
ConstDoublePtr;

class FinPlugin : public ModelPlugin
{
    /// \brief Constructor
    public: FinPlugin();

    /// \brief Destructor
    public: virtual ~FinPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback for the input topic subscriber
    protected: void UpdateInput(ConstDoublePtr &_msg);

    /// \brief Reads current velocity topic
    protected: void UpdateCurrentVelocity(ConstVector3dPtr &_msg);

    /// \brief Fin dynamic model
    protected: std::shared_ptr<Dynamics> dynamics;

    /// \brief Lift&Drag model
    protected: std::shared_ptr<LiftDrag> liftdrag;

    /// \brief Update event
    protected: event::ConnectionPtr updateConnection;

    /// \brief Gazebo node
    protected: transport::NodePtr node;

    /// \brief The fin joint
    protected: physics::JointPtr joint;

    /// \brief The fin link
    protected: physics::LinkPtr link;

    /// \brief Subscriber to the reference signal topic.
    protected: transport::SubscriberPtr commandSubscriber;

    /// \brief Publisher to the output thrust topic
    protected: transport::PublisherPtr anglePublisher;

    /// \brief Force component calculated from the lift and drag module
    protected: ignition::math::Vector3d finForce;

    /// \brief Latest input command.
    protected: double inputCommand;

    /// \brief Fin ID
    protected: int finID;

    /// \brief Topic prefix
    protected: std::string topicPrefix;

    /// \brief Latest fin angle in [rad].
    protected: double angle;

    /// \brief Time stamp of latest thrust force
    protected: common::Time angleStamp;

    /// \brief Subcriber to current message
    protected: transport::SubscriberPtr currentSubscriber;

    /// \brief Current velocity vector read from topic
    protected: ignition::math::Vector3d currentVelocity;
};
}

#endif
