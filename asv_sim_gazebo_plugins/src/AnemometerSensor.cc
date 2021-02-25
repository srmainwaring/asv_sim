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
#include "asv_sim_gazebo_plugins/MessageTypes.hh"
#include "asv_sim_gazebo_plugins/Utilities.hh"

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

using namespace asv;
using namespace gazebo;
using namespace sensors;

///////////////////////////////////////////////////////////////////////////////
// AnemometerSensorPrivate

// GZ_REGISTER_STATIC_SENSOR("anemometer", AnemometerSensor)

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Create a new AnemometerSensor.
    Sensor* NewAnemometerSensor()
    {
      return new gazebo::sensors::AnemometerSensor();
    }

    /// \brief Register an AnemometerSensor with the gazebo SensorFactory.
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
      public: asv_msgs::msgs::Anemometer anemometerMsg;
    };
  } // sensors
} // gazebo

///////////////////////////////////////////////////////////////////////////////
// AnemometerSensor

AnemometerSensor::~AnemometerSensor()
{
  // Clean up.
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

  this->dataPtr->anemometerPub = this->node->Advertise<asv_msgs::msgs::Anemometer>(
      this->GetTopic(), 50);
}

void AnemometerSensor::Init()
{
  Sensor::Init();
}

void AnemometerSensor::Fini()
{
  this->dataPtr->anemometerPub.reset();
  this->dataPtr->parentLink.reset();
  Sensor::Fini();
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
    msgs::Set(this->dataPtr->anemometerMsg.mutable_wind_velocity(),
      apparentWindRelativeLinearVel);

    // DEBUG

    // gzmsg << "parent_link:            " << this->dataPtr->parentLink->GetName() << "\n";
    // gzmsg << "sensor_link:            " << this->Name() << "\n";
    // gzmsg << "sensor_world_pose:      " << sensorWorldPose << "\n";
    // gzmsg << "link_world_com_lin_vel: " << linkWorldCoMLinearVel << "\n";
    // gzmsg << "link_world_ang_vel:     " << linkWorldAngularVel << "\n";
    // gzmsg << "sensor_com_pose:        " << sensorCoMPose << "\n";
    // gzmsg << "sensor_world_lin_vel:   " << sensorWorldLinearVel << "\n";
    // gzmsg << "wind_world_linear_vel:  " << windWorldLinearVel << "\n";
    // gzmsg << "app_wind_world_lin_vel: " << apparentWindWorldLinearVel << "\n";
    // gzmsg << "app_wind_rel_lin_vel:   " << apparentWindRelativeLinearVel << "\n\n";
  }

  // Save the time of the measurement
  common::Time simTime = this->world->SimTime();
  msgs::Set(this->dataPtr->anemometerMsg.mutable_time(), simTime);

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
  ignition::math::Vector3d vel(
      this->dataPtr->anemometerMsg.wind_velocity().x(),
      this->dataPtr->anemometerMsg.wind_velocity().y(),
      this->dataPtr->anemometerMsg.wind_velocity().z()
  );
  return vel;
}

