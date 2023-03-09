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
#include <gz/sensors/Util.hh>
#include <gz/transport.hh>


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
  this->pub = this->node.Advertise<gz::msgs::Double>(this->Topic());

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

#if 0
  /// \brief A map of air speed entity to its sensor
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::AirSpeedSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  public: Entity entity;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _airSpeed AirSpeedSensor component.
  /// \param[in] _parent Parent entity component.
  public: void AddAirSpeed(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::AirSpeedSensor *_airSpeed,
    const components::ParentEntity *_parent);

  /// \brief Create air speed sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update air speed sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateAirSpeeds(const EntityComponentManager &_ecm);

  /// \brief Remove air speed sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveAirSpeedEntities(const EntityComponentManager &_ecm);
#endif
};

/////////////////////////////////////////////////
Anemometer::~Anemometer() = default;

/////////////////////////////////////////////////
Anemometer::Anemometer()
  : System(), dataPtr(std::make_unique<AnemometerPrivate>())

{
}

/////////////////////////////////////////////////
void Anemometer::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  /// \todo(srmainwaring) implement
#if 0
  physics::EntityPtr parentEntity =
    this->world->EntityByName(this->ParentName());

  this->dataPtr->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  auto getTopic = [&, this]() -> std::string
  {
    std::string topicName = "~/" + this->ParentName() + '/' + this->Name();
    if (this->sdf->HasElement("topic"))
      topicName += '/' + this->sdf->Get<std::string>("topic");
    boost::replace_all(topicName, "::", "/");

    return topicName;
  }

  this->dataPtr->anemometerPub =
      this->node->Advertise<asv_msgs::msgs::Anemometer>(
          getTopic(), 50);

#endif
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
}

/////////////////////////////////////////////////
void Anemometer::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  /// \todo(srmainwaring) implement
}

/////////////////////////////////////////////////
gz::math::Vector3d Anemometer::TrueWindVelocity() const
{
  /// \todo(srmainwaring) implement
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // return gz::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
gz::math::Vector3d Anemometer::ApparentWindVelocity() const
{
  /// \todo(srmainwaring) implement
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // gz::math::Vector3d vel(
  //     this->dataPtr->anemometerMsg.wind_velocity().x(),
  //     this->dataPtr->anemometerMsg.wind_velocity().y(),
  //     this->dataPtr->anemometerMsg.wind_velocity().z());
  // return vel;
  return gz::math::Vector3d::Zero;
}

}  // namespace systems
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::Anemometer,
    gz::sim::System,
    gz::sim::systems::Anemometer::ISystemConfigure,
    gz::sim::systems::Anemometer::ISystemPreUpdate,
    gz::sim::systems::Anemometer::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::Anemometer,
    "gz::sim::systems::Anemometer")
