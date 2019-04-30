/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#include <sensor_ros_plugins/MagnetometerROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
MagnetometerROSPlugin::MagnetometerROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
MagnetometerROSPlugin::~MagnetometerROSPlugin()
{ }

/////////////////////////////////////////////////
void MagnetometerROSPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  GetSDFParam<double>(_sdf, "intensity", this->parameters.intensity, 1.0);
  GetSDFParam<double>(_sdf, "reference_heading", this->parameters.heading,
    M_PI);
  GetSDFParam<double>(_sdf, "declination", this->parameters.declination, 0.0);
  GetSDFParam<double>(_sdf, "inclination", this->parameters.inclination,
    60.*M_PI/180.);
  GetSDFParam<double>(_sdf, "noise_xy", this->parameters.noiseXY, 1.0);
  GetSDFParam<double>(_sdf, "noise_z", this->parameters.noiseZ, 1.4);
  GetSDFParam<double>(_sdf, "turn_on_bias", this->parameters.turnOnBias, 2.0);

  this->magneticFieldWorld.X() = this->parameters.intensity *
    cos(this->parameters.inclination) *
    cos(this->parameters.heading - this->parameters.declination);
  this->magneticFieldWorld.Y() = this->parameters.intensity *
    cos(this->parameters.inclination) *
    sin(this->parameters.heading - this->parameters.declination);
  this->magneticFieldWorld.Z() = this->parameters.intensity *
    -1 * sin(this->parameters.inclination);

  this->AddNoiseModel("turn_on_bias", this->parameters.turnOnBias);

  // FIXME Add different options for noise amplitude for each noise model
  this->turnOnBias.X() = this->GetGaussianNoise("turn_on_bias",
    this->noiseAmp);
  this->turnOnBias.Y() = this->GetGaussianNoise("turn_on_bias",
    this->noiseAmp);
  this->turnOnBias.Z() = this->GetGaussianNoise("turn_on_bias",
    this->noiseAmp);

  // Initialize ROS message
  if (this->enableLocalNEDFrame)
    this->rosMsg.header.frame_id = tfLocalNEDFrame.child_frame_id_;
  else
    this->rosMsg.header.frame_id = this->link->GetName();

  this->AddNoiseModel("noise_xy", this->parameters.noiseXY);
  this->AddNoiseModel("noise_z", this->parameters.noiseZ);

  this->rosMsg.magnetic_field_covariance[0] =
    this->parameters.noiseXY * this->parameters.noiseXY;
  this->rosMsg.magnetic_field_covariance[4] =
    this->parameters.noiseXY * this->parameters.noiseXY;
  this->rosMsg.magnetic_field_covariance[8] =
    this->parameters.noiseZ * this->parameters.noiseZ;

  // Initialize the default magnetometer output
  this->rosSensorOutputPub =
    this->rosNode->advertise<sensor_msgs::MagneticField>(
      this->sensorOutputTopic, 1);

  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Magnetic>(
        this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }
}

/////////////////////////////////////////////////
bool MagnetometerROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  if (!this->EnableMeasurement(_info))
    return false;

  if (this->enableLocalNEDFrame)
    this->SendLocalNEDTransform();

  ignition::math::Pose3d pose;
  // Read sensor link's current pose
#if GAZEBO_MAJOR_VERSION >= 8
  pose = this->link->WorldPose();
#else
  pose = this->link->GetWorldPose().Ign();
#endif

  ignition::math::Vector3d noise(
    this->GetGaussianNoise("noise_xy", this->noiseAmp),
    this->GetGaussianNoise("noise_xy", this->noiseAmp),
    this->GetGaussianNoise("noise_z", this->noiseAmp));

  this->measMagneticField =
    pose.Rot().RotateVectorReverse(this->magneticFieldWorld) +
    this->turnOnBias +
    noise;

  if (this->enableLocalNEDFrame)
    this->measMagneticField = this->localNEDFrame.Rot().RotateVector(
      this->measMagneticField);

  if (this->gazeboMsgEnabled)
  {
    sensor_msgs::msgs::Magnetic gazeboMsg;

    gazebo::msgs::Vector3d* field = new gazebo::msgs::Vector3d();
    field->set_x(this->measMagneticField.X());
    field->set_y(this->measMagneticField.Y());
    field->set_z(this->measMagneticField.Z());
    gazeboMsg.set_allocated_magnetic_field(field);

    this->gazeboSensorOutputPub->Publish(gazeboMsg);
  }

  this->rosMsg.header.stamp = ros::Time::now();
  this->rosMsg.magnetic_field.x = this->measMagneticField.X();
  this->rosMsg.magnetic_field.y = this->measMagneticField.Y();
  this->rosMsg.magnetic_field.z = this->measMagneticField.Z();

  this->rosSensorOutputPub.publish(this->rosMsg);
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(MagnetometerROSPlugin)

}
