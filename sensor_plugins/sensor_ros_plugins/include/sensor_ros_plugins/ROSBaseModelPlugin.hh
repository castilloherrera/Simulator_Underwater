/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __ROS_BASE_MODEL_PLUGIN_HH__
#define __ROS_BASE_MODEL_PLUGIN_HH__

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_ros_plugins/ROSBasePlugin.hh>
#include <functional>
#include <memory>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
  class ROSBaseModelPlugin : public ROSBasePlugin, public ModelPlugin
  {
    /// \brief Class constructor
    public: ROSBaseModelPlugin();

    /// \brief Class destructor
    public: virtual ~ROSBaseModelPlugin();

    /// \brief Load plugin and its configuration from sdf,
    protected: virtual void Load(physics::ModelPtr _model,
      sdf::ElementPtr _sdf);

    /// \brief Update callback from simulation.
    protected: virtual bool OnUpdate(const common::UpdateInfo&);

    /// \brief Pointer to the model.
    protected: physics::ModelPtr model;

    /// \brief Pointer to the link.
    protected: physics::LinkPtr link;

    /// \brief True if a the local NED frame needs to be broadcasted
    protected: bool enableLocalNEDFrame;

    /// \brief TF broadcaster for the local NED frame
    protected: tf::TransformBroadcaster * tfBroadcaster;

    /// \brief Pose of the local NED frame wrt link frame
    protected: ignition::math::Pose3d localNEDFrame;

    /// \brief Local NED TF frame
    protected: tf::StampedTransform tfLocalNEDFrame;

    /// \brief Returns true if the base_link_ned frame exists
    protected: void SendLocalNEDTransform();
  };
}

#endif // __ROS_BASE_MODEL_PLUGIN_HH__
