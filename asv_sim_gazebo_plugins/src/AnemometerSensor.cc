// Copyright (C) 2019  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "asv_sim_gazebo_plugins/AnemometerSensor.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <boost/algorithm/string.hpp>

#include <mutex>
#include <iostream>
#include <string>

using namespace gazebo;
using namespace sensors;

/// \brief Template function for extracting a value from a parameter message.
template <typename T>
void MsgParamSetValue(gazebo::msgs::Param& _param, const T& _value)
{
  gzwarn << "Using default template for MsgParamSetValue" << std::endl;
}

/// \brief Template specialization for setting a bool in a parameter message.
template <>
void MsgParamSetValue<bool>(gazebo::msgs::Param& _param, const bool& _value)
{ 
  _param.mutable_value()->set_bool_value(_value);
}

/// \brief Template specialization for setting an int in a parameter message.
template <>
void MsgParamSetValue<int>(gazebo::msgs::Param& _param, const int& _value)
{ 
  _param.mutable_value()->set_int_value(_value);
}

/// \brief Template specialization for setting a size_t in a parameter message.
template <>
void MsgParamSetValue<size_t>(gazebo::msgs::Param& _param, const size_t& _value)
{ 
  _param.mutable_value()->set_int_value(_value);
}

/// \brief Template specialization for setting a double in a parameter message.
template <>
void MsgParamSetValue<double>(gazebo::msgs::Param& _param, const double& _value)
{ 
  _param.mutable_value()->set_double_value(_value);
}

/// \brief Template specialization for setting a string in a parameter message.
template <>
void MsgParamSetValue<std::string>(gazebo::msgs::Param& _param, const std::string& _value)
{ 
  _param.mutable_value()->set_string_value(_value);
}

/// \brief Template specialization for setting a string in a parameter message.
template <>
void MsgParamSetValue<common::Time>(gazebo::msgs::Param& _param, const common::Time& _value)
{ 
  _param.mutable_value()->mutable_time_value()->set_sec(_value.sec);
  _param.mutable_value()->mutable_time_value()->set_nsec(_value.nsec);
}

/// \brief Template specialization for setting a Vector2 in a parameter message.
// template <>
// void MsgParamSetValue<ignition::math::Vector2d>(gazebo::msgs::Param& _param, const T& _value)
// { 
  // _param.mutable_value()->mutable_vector3d_value()->set_x(_value.x());
  // _param.mutable_value()->mutable_vector3d_value()->set_y(_value.y());
  // _param.mutable_value()->mutable_vector3d_value()->set_z(0);
// }

/// \brief Template specialization for setting a Vector3 in a parameter message.
template <>
void MsgParamSetValue<ignition::math::Vector3d>(gazebo::msgs::Param& _param,
  const ignition::math::Vector3d& _value)
{ 
  _param.mutable_value()->mutable_vector3d_value()->set_x(_value.X());
  _param.mutable_value()->mutable_vector3d_value()->set_y(_value.Y());
  _param.mutable_value()->mutable_vector3d_value()->set_z(_value.Z());
}

/// \brief Template for setting a named parameter on a parameter vector message.
template <typename T>
void SetMsgParam(gazebo::msgs::Param_V& _msg, const std::string &_paramName, const T& _value)
{
  // Custom compare for params
  auto compare = [=](auto& _param)
  {
    return _param.name() == _paramName;
  };

  auto it = std::find_if(std::begin(_msg.param()), std::end(_msg.param()), compare);
  
  // Not found
  if (it == std::end(_msg.param()))
  {
    gzwarn << "Parameter <" << _paramName << "> not found: " 
      <<  "Cannot set to " << _value << std::endl;
    return;
  }

  // Found
  auto index = std::distance(std::begin(_msg.param()), it);
  auto param = _msg.mutable_param(index);
  MsgParamSetValue<T>(*param, _value);

}

///////////////////////////////////////////////////////////////////////////////
// AnemometerSensorPrivate

// GZ_REGISTER_STATIC_SENSOR("anemometer", AnemometerSensor)

namespace gazebo
{
  namespace sensors
  {
    Sensor* NewAnemometerSensor()
    {
      return new gazebo::sensors::AnemometerSensor();
    }

    void RegisterAnemometerSensor()
    {
      SensorFactory::RegisterSensor("anemometer", NewAnemometerSensor);
    }

    /// \internal
    /// \brief Private data for the AnemometerSensor
    class AnemometerSensorPrivate
    {
      /// \brief Mutex to protect read and writes
      public: std::mutex mutex;

      /// \brief Publish to topic "~/anemometer".
      public: transport::PublisherPtr anemometerPub;

      /// \brief Parent link of this sensor.
      public: physics::LinkPtr parentLink;

      /// \brief Store the most recent anemometer message.
      public: msgs::Param_V anemometerMsg;
    };
  } // sensors
} // gazebo

///////////////////////////////////////////////////////////////////////////////
// AnemometerSensor

AnemometerSensor::~AnemometerSensor()
{
  // Clean up.
  this->dataPtr->anemometerPub.reset();
  this->Fini();
}

AnemometerSensor::AnemometerSensor() : 
  Sensor(sensors::OTHER), 
  dataPtr(new AnemometerSensorPrivate())
{
}

void AnemometerSensor::Load(const std::string& _worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

void AnemometerSensor::Load(const std::string& _worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity =
    this->world->EntityByName(this->ParentName());
  
  this->dataPtr->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->dataPtr->anemometerPub = this->node->Advertise<msgs::Param_V>(
      this->GetTopic(), 50);

  // Parse sdf noise parameters
  // sdf::ElementPtr magElem = this->sdf->GetElement("anemometer");

  // if (!magElem)
  // {
  //   gzerr << "Missing <anemometer> element in sensor["
  //     << this->Name() << "]\n";
  // }
  // else
  // // Load anemometer field noise parameters
  // {
  //   if (magElem->HasElement("x") &&
  //       magElem->GetElement("x")->HasElement("noise"))
  //   {
  //     this->noises[ANEMOMETER_X_NOISE] =
  //       NoiseFactory::NewNoiseModel(
  //           magElem->GetElement("x")->GetElement("noise"));
  //   }

  //   if (magElem->HasElement("y") &&
  //       magElem->GetElement("y")->HasElement("noise"))
  //   {
  //     this->noises[ANEMOMETER_Y_NOISE] =
  //       NoiseFactory::NewNoiseModel(
  //           magElem->GetElement("y")->GetElement("noise"));
  //   }

  //   if (magElem->HasElement("z") &&
  //       magElem->GetElement("z")->HasElement("noise"))
  //   {
  //     this->noises[ANEMOMETER_Z_NOISE] =
  //       NoiseFactory::NewNoiseModel(
  //           magElem->GetElement("z")->GetElement("noise"));
  //   }
  // }

  // Initialise the anemometer message
  { // time
    auto nextParam = this->dataPtr->anemometerMsg.add_param();
    nextParam->set_name("time");
    nextParam->mutable_value()->set_type(msgs::Any::TIME);
  }
  { // true wind
    auto nextParam = this->dataPtr->anemometerMsg.add_param();
    nextParam->set_name("true_wind");
    nextParam->mutable_value()->set_type(msgs::Any::VECTOR3D);
  }
  { // apparent wind
    auto nextParam = this->dataPtr->anemometerMsg.add_param();
    nextParam->set_name("apparent_wind");
    nextParam->mutable_value()->set_type(msgs::Any::VECTOR3D);
  }

  // @DEBUG_INFO
  {
    auto nextParam = this->dataPtr->anemometerMsg.add_param();
    nextParam->set_name("link_vel");
    nextParam->mutable_value()->set_type(msgs::Any::VECTOR3D);
  }
  {
    auto nextParam = this->dataPtr->anemometerMsg.add_param();
    nextParam->set_name("link_omega");
    nextParam->mutable_value()->set_type(msgs::Any::VECTOR3D);
  }
  {
    auto nextParam = this->dataPtr->anemometerMsg.add_param();
    nextParam->set_name("sensor_xr");
    nextParam->mutable_value()->set_type(msgs::Any::VECTOR3D);
  }
  {
    auto nextParam = this->dataPtr->anemometerMsg.add_param();
    nextParam->set_name("sensor_vel");
    nextParam->mutable_value()->set_type(msgs::Any::VECTOR3D);
  }

}

void AnemometerSensor::Init()
{
  Sensor::Init();
}

void AnemometerSensor::Fini()
{
  Sensor::Fini();
  this->dataPtr->parentLink.reset();
}

std::string AnemometerSensor::GetTopic() const
{
  std::string topicName = "~/" + this->ParentName() + '/' + this->Name();
  if (this->sdf->HasElement("topic"))
    topicName += '/' + this->sdf->Get<std::string>("topic");
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

bool AnemometerSensor::UpdateImpl(const bool _force)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Get latest pose information
  if (this->dataPtr->parentLink)
  {
    // Sensor pose relative to the world frame
    ignition::math::Pose3d sensorWorldPose
      = this->pose + this->dataPtr->parentLink->WorldPose();

    // Link velocity at the link CoM in the world frame.
    ignition::math::Vector3d linkWorldCoMLinearVel
      = this->dataPtr->parentLink->WorldCoGLinearVel();

    // Link angular velocity at the link CoM in the world frame.
    ignition::math::Vector3d linkWorldAngularVel
      = this->dataPtr->parentLink->WorldAngularVel();

    // Sensor pose relative to the link CoM
    ignition::math::Pose3d sensorCoMPose
      = sensorWorldPose - this->dataPtr->parentLink->WorldCoGPose();

    // Sensor velocity: vs = omega x r_(CoM, sensor)
    ignition::math::Vector3d sensorWorldLinearVel
      = linkWorldCoMLinearVel + linkWorldAngularVel.Cross(sensorCoMPose.Pos());

    // Wind velocity at the link origin in the world frame.
    // We use this to approximate the true wind at the sensor origin
    // (true wind = unadjusted for the sensors's motion)
    auto& wind = this->world->Wind();
    ignition::math::Vector3d windWorldLinearVel
      = wind.WorldLinearVel(this->dataPtr->parentLink.get());

    // Apparent wind velocity at the sensor origin in the world frame.
    ignition::math::Vector3d apparentWindWorldLinearVel 
      = windWorldLinearVel - sensorWorldLinearVel;

    // Apparent wind velocity at the sensor origin in the sensor frame.
    // This is what would be measured by an anemometer.
    ignition::math::Vector3d apparentWindRelativeLinearVel
     = sensorWorldPose.Rot().Inverse().RotateVector(
        apparentWindWorldLinearVel);

    // Update the messages.
    SetMsgParam(this->dataPtr->anemometerMsg,
      "true_wind", windWorldLinearVel);
    SetMsgParam(this->dataPtr->anemometerMsg,
      "apparent_wind", apparentWindRelativeLinearVel);

    // @DEBUG_INFO
    SetMsgParam(this->dataPtr->anemometerMsg,
      "link_vel", linkWorldCoMLinearVel);
    SetMsgParam(this->dataPtr->anemometerMsg,
      "link_omega", linkWorldAngularVel);

    SetMsgParam(this->dataPtr->anemometerMsg,
      "sensor_xr", sensorCoMPose.Pos());
    SetMsgParam(this->dataPtr->anemometerMsg,
      "sensor_vel", sensorWorldLinearVel);

  }

  // Save the time of the measurement
  common::Time simTime = this->world->SimTime();
  SetMsgParam(this->dataPtr->anemometerMsg, "time", simTime);

  // Publish the message if needed
  if (this->dataPtr->anemometerPub)
    this->dataPtr->anemometerPub->Publish(this->dataPtr->anemometerMsg);

  return true;    
}

ignition::math::Vector3d AnemometerSensor::TrueWindVelocity() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return ignition::math::Vector3d::Zero;
}

ignition::math::Vector3d AnemometerSensor::ApparentWindVelocity() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return ignition::math::Vector3d::Zero;
}

