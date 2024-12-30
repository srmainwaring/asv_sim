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

#include "SailPositionController.hh"

#include <gz/msgs/double.pb.h>

#include <atomic>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
namespace systems
{
/////////////////////////////////////////////////
class SailPositionControllerPrivate
{
  /// \brief Callback for position subscription
  /// \param[in] _msg Position message
  public: void OnCmdPos(const msgs::Double &_msg);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: std::vector<Entity> jointEntities;

  /// \brief Joint name
  public: std::vector<std::string> jointNames;

  /// \brief Commanded joint position
  public: std::atomic<double> jointPosCmd{0.0};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Position PID controller.
  public: math::PID posPid;

  /// \brief Joint index to be used.
  public: unsigned int jointIndex{0};
};

/////////////////////////////////////////////////
void SailPositionControllerPrivate::OnCmdPos(const msgs::Double &_msg)
{
  this->jointPosCmd = _msg.data();
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
SailPositionController::~SailPositionController() = default;

/////////////////////////////////////////////////
SailPositionController::SailPositionController()
  : System(), dataPtr(std::make_unique<SailPositionControllerPrivate>())
{
}

/////////////////////////////////////////////////
void SailPositionController::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "SailPositionController plugin should be attached to a model "
          << "entity. Failed to initialize." << "\n";
    return;
  }

  // Get params from SDF
  auto sdfElem = _sdf->FindElement("joint_name");
  while (sdfElem)
  {
    if (!sdfElem->Get<std::string>().empty())
    {
      this->dataPtr->jointNames.push_back(sdfElem->Get<std::string>());
    }
    else
    {
      gzerr << "<joint_name> provided but is empty." << "\n";
    }
    sdfElem = sdfElem->GetNextElement("joint_name");
  }
  if (this->dataPtr->jointNames.empty())
  {
    gzerr << "Failed to get any <joint_name>." << "\n";
    return;
  }

  if (_sdf->HasElement("joint_index"))
  {
    this->dataPtr->jointIndex = _sdf->Get<unsigned int>("joint_index");
  }

  // PID parameters
  double p         =  1;
  double i         =  0.1;
  double d         =  0.01;
  double iMax      =  1;
  double iMin      = -1;
  double cmdMax    =  1000;
  double cmdMin    = -1000;
  double cmdOffset =  0;

  if (_sdf->HasElement("p_gain"))
  {
    p = _sdf->Get<double>("p_gain");
  }
  if (_sdf->HasElement("i_gain"))
  {
    i = _sdf->Get<double>("i_gain");
  }
  if (_sdf->HasElement("d_gain"))
  {
    d = _sdf->Get<double>("d_gain");
  }
  if (_sdf->HasElement("i_max"))
  {
    iMax = _sdf->Get<double>("i_max");
  }
  if (_sdf->HasElement("i_min"))
  {
    iMin = _sdf->Get<double>("i_min");
  }
  if (_sdf->HasElement("cmd_max"))
  {
    cmdMax = _sdf->Get<double>("cmd_max");
  }
  if (_sdf->HasElement("cmd_min"))
  {
    cmdMin = _sdf->Get<double>("cmd_min");
  }
  if (_sdf->HasElement("cmd_offset"))
  {
    cmdOffset = _sdf->Get<double>("cmd_offset");
  }

  this->dataPtr->posPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);


  if (_sdf->HasElement("initial_position"))
  {
    this->dataPtr->jointPosCmd = _sdf->Get<double>("initial_position");
  }

  // Subscribe to commands
  std::string topic;
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic")))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
        this->dataPtr->model.Name(_ecm) + "/joint/" +
        this->dataPtr->jointNames[0] + "/" +
        std::to_string(this->dataPtr->jointIndex) + "/cmd_pos");
    if (topic.empty())
    {
      gzerr << "Failed to create topic for joint ["
            << this->dataPtr->jointNames[0]
            << "]" << "\n";
      return;
    }
  }
  if (_sdf->HasElement("sub_topic"))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/" +
        _sdf->Get<std::string>("sub_topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic from sub_topic [/model/"
            << this->dataPtr->model.Name(_ecm) << "/"
            << _sdf->Get<std::string>("sub_topic")
            << "]" << " for joint [" << this->dataPtr->jointNames[0]
            << "]" << "\n";
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
            << "]" << " for joint [" << this->dataPtr->jointNames[0]
            << "]" << "\n";
      return;
    }
  }
  this->dataPtr->node.Subscribe(
      topic, &SailPositionControllerPrivate::OnCmdPos, this->dataPtr.get());

  gzdbg << "[SailPositionController] system parameters:" << "\n"
        << "p_gain: ["     << p         << "]"            << "\n"
        << "i_gain: ["     << i         << "]"            << "\n"
        << "d_gain: ["     << d         << "]"            << "\n"
        << "i_max: ["      << iMax      << "]"            << "\n"
        << "i_min: ["      << iMin      << "]"            << "\n"
        << "cmd_max: ["    << cmdMax    << "]"            << "\n"
        << "cmd_min: ["    << cmdMin    << "]"            << "\n"
        << "cmd_offset: [" << cmdOffset << "]"            << "\n"
        << "topic: ["      << topic     << "]"            << "\n"
        << "initial_position: [" << this->dataPtr->jointPosCmd << "]"
        << "\n";
}

/////////////////////////////////////////////////
void SailPositionController::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SailPositionController::PreUpdate");

  /// \todo(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << "\n";
  }

  // If the joints haven't been identified yet, look for them
  if (this->dataPtr->jointEntities.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->jointNames)
    {
      // First try to resolve by scoped name.
      Entity joint = kNullEntity;
      auto entities = entitiesFromScopedName(
          name, _ecm, this->dataPtr->model.Entity());

      if (!entities.empty())
      {
        if (entities.size() > 1)
        {
          gzwarn << "Multiple joint entities with name ["
                 << name << "] found. "
                 << "Using the first one.\n";
        }
        joint = *entities.begin();

        // Validate
        if (!_ecm.EntityHasComponentType(joint, components::Joint::typeId))
        {
          gzerr << "Entity with name[" << name
                << "] is not a joint\n";
          joint = kNullEntity;
        }
        else
        {
          gzdbg << "Identified joint [" << name
                << "] as Entity [" << joint << "]\n";
        }
      }

      if (joint != kNullEntity)
      {
        this->dataPtr->jointEntities.push_back(joint);
      }
      else if (!warned)
      {
        gzwarn << "Failed to find joint [" << name << "]\n";
        warned = true;
      }
    }
  }
  if (this->dataPtr->jointEntities.empty())
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Create joint position component if one doesn't exist
  auto jointPosComp = _ecm.Component<components::JointPosition>(
      this->dataPtr->jointEntities[0]);
  if (!jointPosComp)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntities[0],
        components::JointPosition());
  }

  // We just created the joint position component, give one iteration for the
  // physics system to update its size
  if (jointPosComp == nullptr || jointPosComp->Data().empty())
    return;

  // Sanity check: Make sure that the joint index is valid.
  if (this->dataPtr->jointIndex >= jointPosComp->Data().size())
  {
    static std::unordered_set<Entity> reported;
    if (reported.find(this->dataPtr->jointEntities[0]) == reported.end())
    {
      gzerr << "[SailPositionController]: Detected an invalid <joint_index> "
             << "parameter. The index specified is ["
             << this->dataPtr->jointIndex << "] but joint ["
             << this->dataPtr->jointNames[0] << "] only has ["
             << jointPosComp->Data().size() << "] index[es]. "
             << "This controller will be ignored" << "\n";
      reported.insert(this->dataPtr->jointEntities[0]);
    }
    return;
  }

  // Get error in position
  const double pos = jointPosComp->Data().at(this->dataPtr->jointIndex);
  const double pos_sgn = pos < 0.0 ? -1.0 : 1.0;

  // Target position will be in [0, pos_max] (positive),
  // we set it to have the same sign as the current position
  const double pos_target = pos_sgn * this->dataPtr->jointPosCmd;

  // Calculate the error
  const double error = pos - pos_target;

  for (Entity joint : this->dataPtr->jointEntities)
  {
    // Update force command.
    double force = this->dataPtr->posPid.Update(error, _info.dt);

    // Only apply tension forces (when |pos_target| < |pos|)
    if (force * pos_sgn > 0)
    {
      force = 0.0;
    }

    auto forceComp = _ecm.Component<components::JointForceCmd>(joint);
    if (forceComp == nullptr)
    {
      _ecm.CreateComponent(joint, components::JointForceCmd({force}));
    }
    else
    {
      *forceComp = components::JointForceCmd({force});
    }
  }
}

}  // namespace systems
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::SailPositionController,
    gz::sim::System,
    gz::sim::systems::SailPositionController::ISystemConfigure,
    gz::sim::systems::SailPositionController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::SailPositionController,
    "gz::sim::systems::SailPositionController")
