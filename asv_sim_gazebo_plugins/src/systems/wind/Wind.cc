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


// Portions of this code are modified from the Gazebo JointPositionController
/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "Wind.hh"

#include <gz/msgs/vector3d.pb.h>

#include <mutex>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/World.hh>

#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
namespace systems
{
/////////////////////////////////////////////////
class WindPrivate
{
  /// \brief Callback for position subscription
  /// \param[in] _msg Position message
  public: void OnWindVelocity(const msgs::Vector3d &_msg);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Mutex windVel
  public: std::mutex windVelocityMutex;

  /// \brief Initialization flag
  public: bool initialized{false};

  /// \brief World
  public: World world;

  /// \brief World wind velocity 
  public: math::Vector3d windVelocityWorld;
};

/////////////////////////////////////////////////
void WindPrivate::OnWindVelocity(const msgs::Vector3d &_msg)
{
  std::lock_guard<std::mutex> lock(this->windVelocityMutex);
  this->windVelocityWorld = msgs::Convert(_msg);
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
Wind::~Wind() = default;

/////////////////////////////////////////////////
Wind::Wind()
  : System(), dataPtr(std::make_unique<WindPrivate>())
{
}

/////////////////////////////////////////////////
void Wind::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  this->dataPtr->world = sim::World(_entity);

  if (!this->dataPtr->world.Valid(_ecm))
  {
    gzerr << "Wind plugin should be attached to a world "
          << "entity. Failed to initialize." << "\n";
    return;
  }

  // Subscribe to wind velocity
  std::string topic;
  if (!_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic("/world/" +
        this->dataPtr->world.Name(_ecm).value() + "/wind/vel");
    if (topic.empty())
    {
      gzerr << "Failed to create topic for wind velocity\n";
      return;
    }
  }
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
            << "] for wind velocity\n";
      return;
    }
  }
  this->dataPtr->node.Subscribe(
      topic, &WindPrivate::OnWindVelocity, this->dataPtr.get());

  gzdbg << "[Wind] system parameters:" << "\n"
        << "topic: [" << topic << "]\n"
        << "\n";
}

/////////////////////////////////////////////////
void Wind::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Wind::PreUpdate");

  /// \todo(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << "\n";
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

}

}  // namespace systems
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::Wind,
    gz::sim::System,
    gz::sim::systems::Wind::ISystemConfigure,
    gz::sim::systems::Wind::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::Wind,
    "gz::sim::systems::Wind")
