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

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include <gz/sim/components/CustomSensor.hh>
#include "gz/sim/components/LinearVelocity.hh"
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
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
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
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

        // Enable components (enable velocity checks)
        enableComponent<components::WorldLinearVelocity>(_ecm, _entity, true);
        enableComponent<components::WorldAngularVelocity>(_ecm, _entity, true);
        enableComponent<components::LinearVelocity>(_ecm, _entity, true);
        enableComponent<components::AngularVelocity>(_ecm, _entity, true);
        enableComponent<components::WorldPose>(_ecm, _entity, true);

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
      // Sensor pose relative to the world frame
      math::Pose3d X_WS = worldPose(entity, _ecm);

      /// calculates the expected velocity of the sensor in sensor link frame.
      math::Vector3d v_s_S = relativeVel(entity, _ecm);
      math::Vector3d v_s_W = X_WS.Rot().RotateVector(v_s_S);

      // Wind velocity at the link origin in the world frame.
      // We use this to approximate the true wind at the sensor origin
      // (true wind = unadjusted for the sensors's motion)
      math::Vector3d v_wt_W = math::Vector3d::Zero;
      Entity windEntity = _ecm.EntityByComponents(components::Wind());
      auto velWindWorldComp =
          _ecm.Component<components::WorldLinearVelocity>(windEntity);
      if (velWindWorldComp)
      {
        v_wt_W = velWindWorldComp->Data();
      }

      // Apparent wind velocity at the sensor origin in the world frame.
      math::Vector3d v_wa_W = v_wt_W - v_s_W;

      // Apparent wind velocity at the sensor origin in the sensor frame.
      // This is what would be measured by an anemometer.
      math::Vector3d v_wa_S = X_WS.Rot().Inverse().RotateVector(v_wa_W);

      // debug info
      #if 0
      auto link = sim::Link(entity);
      gzmsg << "\n"
            << "parent_link:  "
            << link.Name(_ecm).value() << "\n"
            << "sensor_link:  " << sensor->Name() << "\n"
            << "X_WS.Pos():   " << X_WS.Pos() << "\n"
            << "X_WS.Rot():   " << X_WS.Rot().Euler() << "\n"
            << "v_s_S:        " << v_s_S << "\n"
            << "v_s_W:        " << v_s_W << "\n"
            << "v_wt_W:       " << v_wt_W << "\n"
            << "v_wa_W:       " << v_wa_W << "\n"
            << "v_wa_S:       " << v_wa_S
            << "\n";
      #endif

      // Update the sensor.
      sensor->SetApparentWindVelocity(v_wa_S);
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
