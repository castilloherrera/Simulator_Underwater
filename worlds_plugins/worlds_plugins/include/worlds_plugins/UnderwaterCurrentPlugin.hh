/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

/// \file UnderwaterCurrentPlugin.hh
/// \brief Plugin that for the underwater world

#ifndef __UNDERWATER_CURRENT_PLUGIN_HH__
#define __UNDERWATER_CURRENT_PLUGIN_HH__

#include <map>
#include <cmath>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <worlds_plugins/GaussMarkovProcess.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief Class for the underwater current plugin
  /// TODO: Add option to make the underwater current also a function of depth
  ///       to comply with DNV
  class UnderwaterCurrentPlugin : public WorldPlugin
  {
    /// \brief Class constructor
    public: UnderwaterCurrentPlugin();

    /// \brief Class destructor
    public: virtual ~UnderwaterCurrentPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world,
        sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void Update(const common::UpdateInfo &_info);

    /// \brief Publish current velocity and the pose of its frame
    protected: void PublishCurrentVelocity();

    /// \brief Update event
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world
    protected: physics::WorldPtr world;

    /// \brief Pointer to sdf
    protected: sdf::ElementPtr sdf;

    /// \brief True if the sea surface is present
    protected: bool hasSurface;

    /// \brief Pointer to a node for communication
    protected: transport::NodePtr node;

    /// \brief Map of publishers
    protected: std::map<std::string, transport::PublisherPtr>
      publishers;

    /// \brief Current velocity topic
    protected: std::string currentVelocityTopic;

    /// \brief Namespace for topics and services
    protected: std::string ns;

    /// \brief Gauss-Markov process instance for the current velocity
    protected: GaussMarkovProcess currentVelModel;

    /// \brief Gauss-Markov process instance for horizontal angle model
    protected: GaussMarkovProcess currentHorzAngleModel;

    /// \brief Gauss-Markov process instance for vertical angle model
    protected: GaussMarkovProcess currentVertAngleModel;

    /// \brief Last update time stamp
    protected: common::Time lastUpdate;

    /// \brief Current linear velocity vector
    protected: ignition::math::Vector3d currentVelocity;
  };
}

#endif  // __UNDERWATER_CURRENT_PLUGIN_HH__
