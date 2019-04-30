/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#include <sensor_ros_plugins/ROSBaseSensorPlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
ROSBaseSensorPlugin::ROSBaseSensorPlugin()
{ }

/////////////////////////////////////////////////
ROSBaseSensorPlugin::~ROSBaseSensorPlugin()
{ }

/////////////////////////////////////////////////
void ROSBaseSensorPlugin::Load(sensors::SensorPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize model pointer
  #if GAZEBO_MAJOR_VERSION >= 7
    this->parentSensor = std::dynamic_pointer_cast<sensors::Sensor>(_model);
  #else
    this->parentSensor = boost::dynamic_pointer_cast<sensors::Sensor>(_model);
  #endif

  // Get the world name.
  std::string worldName = _model->WorldName();
  this->world = physics::get_world(worldName);

  this->InitBasePlugin(_sdf);
}

/////////////////////////////////////////////////
bool ROSBaseSensorPlugin::OnUpdate(const common::UpdateInfo&)
{
  return true;
}

}
