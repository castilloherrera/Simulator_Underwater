/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#ifndef __IMU_ROS_PLUGIN_HH__
#define __IMU_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <sensor_msgs/Imu.h>
#include "SensorImu.pb.h"

// Default values for use with ADIS16448 IMU
#define K_DEFAULT_ADIS_GYROSCOPE_NOISE_DENSITY              2.0 * 35.0 / 3600.0 / 180.0 * M_PI
#define K_DEFAULT_ADIS_GYROSCOPE_RANDOM_WALK                2.0 * 4.0 / 3600.0 / 180.0 * M_PI
#define K_DEFAULT_ADIS_GYROSCOPE_BIAS_CORRELATION_TIME      1.0e+3
#define K_DEFAULT_ADIS_GYROSCOPE_TURN_ON_BIAS_SIGMA         0.5 / 180.0 * M_PI
#define K_DEFAULT_ADIS_ACCELEROMETER_NOISE_DENSITY          2.0 * 2.0e-3
#define K_DEFAULT_ADIS_ACCELEROMETER_RANDOM_WALK            2.0 * 3.0e-3
#define K_DEFAULT_ADIS_ACCELEROMETER_BIAS_CORRELATION_TIME  300.0
#define K_DEFAULT_ADIS_ACCELEROMETER_TURN_ON_BIAS_SIGMA     20.0e-3 * 9.8
#define K_DEFAULT_ORIENTATION_NOISE                         0.5

namespace gazebo
{
  /// \brief IMUParameters stores all IMU model parameters.
  /// A description of these parameters can be found here:
  /// https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
  struct IMUParameters {
    /// \brief Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
    double gyroscopeNoiseDensity;

    /// \brief Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
    double gyroscopeRandomWalk;

    /// \brief Gyroscope bias correlation time constant [s]
    double gyroscopeBiasCorrelationTime;

    /// \brief Gyroscope turn on bias standard deviation [rad/s]
    double gyroscopeTurnOnBiasSigma;

    /// \brief Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
    double accelerometerNoiseDensity;

    /// \brief Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
    double accelerometerRandomWalk;

    /// \brief Accelerometer bias correlation time constant [s]
    double accelerometerBiasCorrelationTime;

    /// \brief Accelerometer turn on bias standard deviation [m/s^2]
    double accelerometerTurnOnBiasSigma;

    /// \brief Standard deviation of orientation noise per axis [rad].
    double orientationNoise;

    /// \brief Constructor.
    IMUParameters()
        : gyroscopeNoiseDensity(
            K_DEFAULT_ADIS_GYROSCOPE_NOISE_DENSITY),
          gyroscopeRandomWalk(
            K_DEFAULT_ADIS_GYROSCOPE_RANDOM_WALK),
          gyroscopeBiasCorrelationTime(
            K_DEFAULT_ADIS_GYROSCOPE_BIAS_CORRELATION_TIME),
          gyroscopeTurnOnBiasSigma(
            K_DEFAULT_ADIS_GYROSCOPE_TURN_ON_BIAS_SIGMA),
          accelerometerNoiseDensity(
            K_DEFAULT_ADIS_ACCELEROMETER_NOISE_DENSITY),
          accelerometerRandomWalk(
            K_DEFAULT_ADIS_ACCELEROMETER_RANDOM_WALK),
          accelerometerBiasCorrelationTime(
            K_DEFAULT_ADIS_ACCELEROMETER_BIAS_CORRELATION_TIME),
          accelerometerTurnOnBiasSigma(
            K_DEFAULT_ADIS_ACCELEROMETER_TURN_ON_BIAS_SIGMA),
          orientationNoise(
            K_DEFAULT_ORIENTATION_NOISE)
    {}
  };

  class IMUROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: IMUROSPlugin();

    /// \brief Class destructor
    public: virtual ~IMUROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Apply and add nosie model to ideal measurements.
    protected: void AddNoise(ignition::math::Vector3d& _linAcc,
                             ignition::math::Vector3d& _angVel,
                             ignition::math::Quaterniond& _orientation,
                             double _dt);

    /// \brief Last measurement of linear acceleration..
    protected: ignition::math::Vector3d measLinearAcc;

    /// \brief Last measurement of angular velocity.
    protected: ignition::math::Vector3d measAngularVel;

    /// \brief (Simulation) time when the last sensor measurement was generated.
    protected: ignition::math::Quaterniond measOrientation;

    /// \brief Gravity vector wrt. reference frame
    protected: ignition::math::Vector3d gravityWorld;

    /// \brief Current (drifting) gyroscope bias.
    protected: ignition::math::Vector3d gyroscopeBias;

    /// \brief Current (drifting) accelerometer bias.
    protected: ignition::math::Vector3d accelerometerBias;

    /// \brief Constant turn-on gyroscope bias.
    protected: ignition::math::Vector3d gyroscopeTurnOnBias;

    /// \brief Constant turn-on accelerometer bias.
    protected: ignition::math::Vector3d accelerometerTurnOnBias;

    /// \brief IMU model parameters.
    protected: IMUParameters imuParameters;

    /// \brief ROS IMU message
    protected: sensor_msgs::Imu imuROSMessage;
  };
}

#endif // __IMU_ROS_PLUGIN_HH__
