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

#include <mutex>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport.hh>

// #include <mutex>
// #include <iostream>
// #include <string>
// #include <boost/algorithm/string.hpp>

// #include <gazebo/common/Assert.hh>
// #include <gazebo/physics/Link.hh>
// #include <gazebo/physics/PhysicsTypes.hh>
// #include <gazebo/physics/World.hh>
// #include <gazebo/sensors/Noise.hh>
// #include <gazebo/sensors/SensorFactory.hh>
// #include <gazebo/transport/Node.hh>
// #include <gazebo/transport/TransportTypes.hh>

// #include <gazebo/msgs/msgs.hh>

// #include <ignition/math/Pose3.hh>
// #include <ignition/math/Vector3.hh>

// #include "asv/sim/MessageTypes.hh"
// #include "asv/sim/Utilities.hh"

// using namespace asv;

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
  public: Entity parentLink;

  /// \todo(srmainwaring) enable
  /// \brief Publish to topic "~/anemometer".
  // public: transport::PublisherPtr anemometerPub;

  /// \brief Communication node.
  public: transport::Node node;

   /// \brief Mutex to protect read and writes
  public: std::mutex mutex;

  /// \todo(srmainwaring) enable
  /// \brief Store the most recent anemometer message.
  // public: asv_msgs::msgs::Anemometer anemometerMsg;
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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  /// \todo(srmainwaring) implement
#if 0
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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return gz::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
gz::math::Vector3d Anemometer::ApparentWindVelocity() const
{
  /// \todo(srmainwaring) implement
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
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
