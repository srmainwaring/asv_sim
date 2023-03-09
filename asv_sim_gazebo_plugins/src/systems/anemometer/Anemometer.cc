// Copyright (C) 2019-2023 Rhys Mainwaring
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

#include "Anemometer.hh"

#include <gz/msgs/vector3d.pb.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include <gz/transport.hh>

#include <sdf/Sensor.hh>

namespace custom
{
//////////////////////////////////////////////////
Anemometer::~Anemometer() = default;

//////////////////////////////////////////////////
bool Anemometer::Load(const sdf::Sensor &_sdf)
{
  auto type = gz::sensors::customType(_sdf);
  if ("anemometer" != type)
  {
    gzerr << "Trying to load [anemometer] sensor, but got type ["
           << type << "] instead.\n";
    return false;
  }

  // Load common sensor params
  gz::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<gz::msgs::Vector3d>(this->Topic());

  if (!_sdf.Element()->HasElement("gz:anemometer"))
  {
    gzdbg << "No custom configuration for [" << this->Topic() << "]\n";
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("gz:anemometer");

  if (!customElem->HasElement("noise"))
  {
    gzdbg << "No noise for [" << this->Topic() << "]\n";
    return true;
  }

  sdf::Noise noiseSdf;
  noiseSdf.Load(customElem->GetElement("noise"));
  this->noise = gz::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    gzerr << "Failed to load noise.\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Anemometer::Update(const std::chrono::steady_clock::duration &_now)
{
  gz::msgs::Vector3d msg;
  *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  gz::msgs::Set(&msg, this->prevApparentWindVel);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void Anemometer::SetApparentWindVelocity(const gz::math::Vector3d &_vel)
{
  this->prevApparentWindVel = _vel;
}

//////////////////////////////////////////////////
const gz::math::Vector3d& Anemometer::ApparentWindVelocity() const
{
  return this->prevApparentWindVel;
}

}  // namespace custom

//////////////////////////////////////////////////
namespace gz
{
namespace sim
{
namespace systems
{

/// \brief Private Anemometer data class.
class AnemometerPrivate
{
  /// \brief Parent link of this sensor.
  // public: Entity parentLink;

  /// \todo(srmainwaring) enable
  /// \brief Publish to topic "~/anemometer".
  // public: transport::PublisherPtr anemometerPub;

  /// \brief Communication node.
  // public: transport::Node node;

   /// \brief Mutex to protect read and writes
  // public: std::mutex mutex;

  /// \todo(srmainwaring) enable
  /// \brief Store the most recent anemometer message.
  // public: asv_msgs::msgs::Anemometer anemometerMsg;

  /// \brief Remove custom sensors if their entities have been removed from
  /// the simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveSensorEntities(
      const gz::sim::EntityComponentManager &_ecm);

  /// \brief A map of custom entities to their sensors.
  public: std::unordered_map<gz::sim::Entity,
      std::shared_ptr<custom::Anemometer>> entitySensorMap;
};

/////////////////////////////////////////////////
void AnemometerPrivate::RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *)->bool
      {
        if (this->entitySensorMap.erase(_entity) == 0)
        {
          gzerr << "Internal error, missing anemometer for entity ["
                << _entity << "].\n";
        }
        return true;
      });
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
Anemometer::~Anemometer() = default;

/////////////////////////////////////////////////
Anemometer::Anemometer()
  : System(), dataPtr(std::make_unique<AnemometerPrivate>())

{
}

/////////////////////////////////////////////////
void Anemometer::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
#if 0
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  /// \todo(srmainwaring) implement
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

    // debug info
    gzmsg << "parent_link:            " << this->dataPtr->parentLink->GetName()
          << "\n";
    gzmsg << "sensor_link:            " << this->Name() << "\n";
    gzmsg << "sensor_world_pose:      " << sensorWorldPose << "\n";
    gzmsg << "link_world_com_lin_vel: " << linkWorldCoMLinearVel << "\n";
    gzmsg << "link_world_ang_vel:     " << linkWorldAngularVel << "\n";
    gzmsg << "sensor_com_pose:        " << sensorCoMPose << "\n";
    gzmsg << "sensor_world_lin_vel:   " << sensorWorldLinearVel << "\n";
    gzmsg << "wind_world_linear_vel:  " << windWorldLinearVel << "\n";
    gzmsg << "app_wind_world_lin_vel: " << apparentWindWorldLinearVel << "\n";
    gzmsg << "app_wind_rel_lin_vel:   " << apparentWindRelativeLinearVel
          << "\n";
    gzmsg << "\n";
  }

  // Save the time of the measurement
  common::Time simTime = this->world->SimTime();
  msgs::Set(this->dataPtr->anemometerMsg.mutable_time(), simTime);

  // Publish the message if needed
  if (this->dataPtr->anemometerPub)
    this->dataPtr->anemometerPub->Publish(this->dataPtr->anemometerMsg);

#endif

  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        // Get sensor's scoped name without the world
        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/anemometer";
          data.SetTopic(topic);
        }

        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<custom::Anemometer>(data);
        if (nullptr == sensor)
        {
          gzerr << "Failed to create anemometer [" << sensorScopedName << "]."
                << "\n";
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<gz::sim::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            gz::sim::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->dataPtr->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        return true;
      });
}

/////////////////////////////////////////////////
void Anemometer::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    for (auto &[entity, sensor] : this->dataPtr->entitySensorMap)
    {
      /// \todo(srmainwaring) implement sensor. 
      // sensor->NewPosition(gz::sim::worldPose(entity, _ecm).Pos());
      sensor->Update(_info.simTime);
    }
  }

  this->dataPtr->RemoveSensorEntities(_ecm);
}

}  // namespace systems
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::Anemometer,
    gz::sim::System,
    gz::sim::systems::Anemometer::ISystemPreUpdate,
    gz::sim::systems::Anemometer::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::Anemometer,
    "gz::sim::systems::Anemometer")
