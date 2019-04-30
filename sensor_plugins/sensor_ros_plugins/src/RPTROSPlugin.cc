/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#include <sensor_ros_plugins/RPTROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
RPTROSPlugin::RPTROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
RPTROSPlugin::~RPTROSPlugin()
{ }

/////////////////////////////////////////////////
void RPTROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  double variance = this->noiseSigma * this->noiseSigma;
  for (int i = 0; i < 9; i++)
    this->rosMessage.pos.covariance[i] = 0;

  this->rosMessage.pos.covariance[0] = this->rosMessage.pos.covariance[4] =
      this->rosMessage.pos.covariance[8] = variance;

  // Initialize the default RPT output
  this->rosSensorOutputPub =
    this->rosNode->advertise<
      sensor_ros_plugins_msgs::PositionWithCovarianceStamped>(
        this->sensorOutputTopic, 1);

  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Rpt>(
        this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }
}

/////////////////////////////////////////////////
bool RPTROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  // True position
  // TODO This is a temporary implementation, next step includes making
  // plugins for acoustic channels and beacons
#if GAZEBO_MAJOR_VERSION >= 8
  this->position = this->link->WorldPose().Pos();
#else
  this->position = this->link->GetWorldPose().Ign().Pos();
#endif

  this->UpdateReferenceFramePose();
  if (this->referenceLink)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->referenceFrame = this->referenceLink->WorldPose();
#else
    this->referenceFrame = this->referenceLink->GetWorldPose().Ign();
#endif
  }

  this->position = this->position - this->referenceFrame.Pos();
  this->position = this->referenceFrame.Rot().RotateVectorReverse(
    this->position);

  this->position.X() += this->GetGaussianNoise(this->noiseAmp);
  this->position.Y() += this->GetGaussianNoise(this->noiseAmp);
  this->position.Z() += this->GetGaussianNoise(this->noiseAmp);

  this->rosMessage.header.stamp = ros::Time::now();
  this->rosMessage.header.frame_id = this->referenceFrameID;
  this->rosMessage.pos.pos.x = this->position.X();
  this->rosMessage.pos.pos.y = this->position.Y();
  this->rosMessage.pos.pos.z = this->position.Z();

  this->rosSensorOutputPub.publish(this->rosMessage);

  if (this->gazeboMsgEnabled)
  {
    sensor_msgs::msgs::Rpt gazeboMessage;
    double variance = this->noiseSigma * this->noiseSigma;
    // Prepare constant covariance part of message
    for (int i = 0 ; i < 9; i++)
    {
      if (i == 0 || i == 4 || i == 8)
        gazeboMessage.add_position_covariance(variance);
      else
        gazeboMessage.add_position_covariance(0.0);
    }
    // Publish simulated measurement
    gazebo::msgs::Vector3d * p = new gazebo::msgs::Vector3d();
    p->set_x(this->position.X());
    p->set_y(this->position.Y());
    p->set_z(this->position.Z());

    gazeboMessage.set_allocated_position(p);
    this->gazeboSensorOutputPub->Publish(gazeboMessage);
  }
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(RPTROSPlugin)
}
