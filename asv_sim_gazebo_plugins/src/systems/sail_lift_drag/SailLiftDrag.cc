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


// Portions of this code are modified from the OSRF LiftDragPlugin
/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "SailLiftDrag.hh"

#include <mutex>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "asv_sim/LiftDragModel.hh"

namespace gz
{
namespace sim
{
namespace systems
{
/////////////////////////////////////////////////
class SailLiftDragPrivate
{
  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Link interface.
  public: Link link{kNullEntity};;

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Publish to topic "/model/<model>/link/<link>/sail_lift_drag".
  public: transport::Node::Publisher liftDragPub;

  /// \brief Update period calculated from <update_rate>.
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Previous update time for publisher throttle.
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief Center of pressure in link local coordinates.
  public: gz::math::Vector3d cp = gz::math::Vector3d::Zero;

  /// \brief Lift drag model.
  public: std::unique_ptr<asv::LiftDragModel> liftDrag;
};

/////////////////////////////////////////////////
SailLiftDrag::~SailLiftDrag() = default;

/////////////////////////////////////////////////
SailLiftDrag::SailLiftDrag()
  : System(), dataPtr(std::make_unique<SailLiftDragPrivate>())
{
}

/////////////////////////////////////////////////
void SailLiftDrag::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "SailLiftDrag plugin should be attached to a model "
           << "entity. Failed to initialize.\n";
    return;
  }

  // Parameters
  {
    if (!_sdf->HasElement("link_name"))
    {
      gzerr << "You must specify a <link_name> for the SailLiftDrag"
            << " plugin to act upon.\n";
      return;
    }
    auto linkName = _sdf->Get<std::string>("link_name");
    auto linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
    if (!_ecm.HasEntity(linkEntity))
    {
      gzerr << "Link with name [" << linkName << "] not found "
            << "in model [" << this->dataPtr->model.Name(_ecm) << "]. "
            << "The SailLiftDrag plugin will not generate forces.\n";
      return;
    }
    this->dataPtr->link = Link(linkEntity);
  }

  {
    if (!_sdf->HasElement("cp"))
    {
      gzerr << "You must specify a <cp> for the SailLiftDrag"
            << " plugin forces to act at.\n";
      return;
    }
    this->dataPtr->cp = _sdf->Get<gz::math::Vector3d>("cp");
  }

  {
    double rate = 1.0;
    std::chrono::duration<double> period{rate > 0.0 ? 1.0 / rate : 0.0};
    this->dataPtr->updatePeriod = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period);
  }

  /// \todo(srmainwaring) implement publishing forces.
#if 0
  auto getTopic = [&, this]() -> std::string
  {
    std::string topic;

    this->LoadParam(this->dataPtr->sdf, "topic", topic, "lift_drag");

    std::string topicName = "~";
    if (this->dataPtr->link != nullptr)
    {
      topicName += '/' + this->dataPtr->link->GetName();
    }
    topicName += '/' + topic;
    boost::replace_all(topicName, "::", "/");

    return topicName;
  }

  // Publishers
  std::string topic = this->GetTopic();
  this->dataPtr->liftDragPub
    = this->dataPtr->node->Advertise<asv_msgs::msgs::LiftDrag>(topic);

  // Time
  this->dataPtr->prevTime = this->dataPtr->world->SimTime();
#endif

  // Lift / Drag model
  this->dataPtr->liftDrag.reset(asv::LiftDragModel::Create(_sdf));
}

/////////////////////////////////////////////////
void SailLiftDrag::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->dataPtr->link.Valid(_ecm))
    return;

  // ensure components are available
  this->dataPtr->link.EnableVelocityChecks(_ecm, true);
  this->dataPtr->link.EnableAccelerationChecks(_ecm, true);

  /// \todo(srmainwaring) get wind model accounting for wind effects plugin
  // retrieve the wind velocity
  auto velWind = math::Vector3d::Zero;
  Entity windEntity = _ecm.EntityByComponents(components::Wind());
  auto windLinearVelComp =
      _ecm.Component<components::WorldLinearVelocity>(windEntity);
  if (windLinearVelComp)
  {
    velWind = windLinearVelComp->Data();
  }

  // Linear velocity at the centre of pressure (world frame).
  auto velCpComp = this->dataPtr->link.WorldLinearVelocity(
      _ecm, this->dataPtr->cp);
  if (!velCpComp.has_value())
    return;
  auto velCp = velCpComp.value();

  // Free stream velocity at centre of pressure (world frame).
  auto vel = velWind - velCp;

  // Pose of link origin and link CoM (world frame).
  auto linkPoseOpt = this->dataPtr->link.WorldPose(_ecm);
  if (!linkPoseOpt.has_value())
    return;
  auto linkPose = linkPoseOpt.value();

  auto comPoseOpt  = this->dataPtr->link.WorldInertialPose(_ecm);
  if (!comPoseOpt.has_value())
    return;
  auto comPose = comPoseOpt.value();

  // Compute lift and drag
  double alpha = 0;
  double u = 0;
  double cl = 0;
  double cd = 0;
  gz::math::Vector3d lift = gz::math::Vector3d::Zero;
  gz::math::Vector3d drag = gz::math::Vector3d::Zero;
  this->dataPtr->liftDrag->Compute(vel, linkPose, lift, drag, alpha, u, cl, cd);

  // Resultant force arising from lift and drag (world frame).
  auto force = lift + drag;

  // CoM (world frame).
  auto com = comPose.Pos();

  // Rotate the centre of pressure (CP) into the world frame.
  auto cpWorld = linkPose.Rot().RotateVector(this->dataPtr->cp);

  // Vector from CoM to CP.
  auto xr = linkPose.Pos() + cpWorld - com;

  // Compute torque (about CoM in world frame)
  auto torque = xr.Cross(force);

  // Ensure no overflow.
  force.Correct();
  torque.Correct();

  // @DEBUG_INFO
#if 1
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed >= this->dataPtr->updatePeriod)
  {
    this->dataPtr->lastUpdateTime = _info.simTime;
    gzmsg << "Link:         " << this->dataPtr->link.Name(_ecm).value() << "\n"
    // Scalars
          << "u:            " << u                  << "\n"
          << "alpha:        " << alpha              << "\n"
          << "cl:           " << cl                 << "\n"
          << "cd:           " << cd                 << "\n"
          << "||velWind||:  " << velWind.Length()   << "\n"
          << "||velCp||:    " << velCp.Length()     << "\n"
          << "||vel||:      " << vel.Length()       << "\n"
          << "||lift||:     " << lift.Length()      << "\n"
          << "||drag||:     " << drag.Length()      << "\n"
          << "||xr||:       " << xr.Length()        << "\n"
          << "||force||:    " << force.Length()     << "\n"
    // Vectors
          << "link.pos:     " << linkPose.Pos()     << "\n"
          << "link.rot:     " << linkPose.Rot().Euler() << "\n"
          << "com.pos:      " << comPose.Pos()     << "\n"
          << "com.rot:      " << comPose.Rot().Euler() << "\n"
          << "velWind:      " << velWind            << "\n"
          << "velCp:        " << velCp              << "\n"
          << "vel:          " << vel                << "\n"
          << "lift:         " << lift               << "\n"
          << "drag:         " << drag               << "\n"
          << "xr:           " << xr                 << "\n"
          << "force:        " << force              << "\n"
          << "torque:       " << torque             << "\n"
          << "link          " << linkPose.Pos()     << "\n"
          << "com:          " << com                << "\n"
          << "cp:           " << cpWorld            << "\n"
          << "\n";
  }
#endif

  // Add force and torque to link (applied to CoM in world frame).
  this->dataPtr->link.AddWorldForce(_ecm, force, com);
  this->dataPtr->link.AddWorldWrench(_ecm, math::Vector3d::Zero, torque);

  /// \todo(srmainwaring) enable force publishing / visualization.
#if 0
  // Publish message
  const double updateRate = 50.0;
  const double updateInterval = 1.0/updateRate;
  common::Time simTime = this->dataPtr->world->SimTime();
  if ((simTime - this->dataPtr->prevTime).Double() < updateInterval)
  {
    return;
  }
  this->dataPtr->prevTime = simTime;

  // Save the time of the measurement
  asv_msgs::msgs::LiftDrag liftDragMsg;
  msgs::Set(liftDragMsg.mutable_time(), simTime);

  msgs::Set(liftDragMsg.mutable_lift(), lift);
  msgs::Set(liftDragMsg.mutable_drag(), drag);

  liftDragMsg.set_alpha(alpha);
  liftDragMsg.set_speed_ld(u);
  liftDragMsg.set_lift_coeff(cl);
  liftDragMsg.set_drag_coeff(cd);

  msgs::Set(liftDragMsg.mutable_vel(), vel);
  // msgs::Set(liftDragMsg.mutable_vel_ld(), velLD);
  msgs::Set(liftDragMsg.mutable_xr(), xr);
  msgs::Set(liftDragMsg.mutable_force(), force);
  msgs::Set(liftDragMsg.mutable_torque(), torque);

  // Publish the message if needed
  if (this->dataPtr->liftDragPub)
    this->dataPtr->liftDragPub->Publish(liftDragMsg);
  // @DEBUG_INFO
  // gzmsg << liftDragMsg.DebugString() << "\n";
#endif
}

}  // namespace systems
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::SailLiftDrag,
    gz::sim::System,
    gz::sim::systems::SailLiftDrag::ISystemConfigure,
    gz::sim::systems::SailLiftDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::SailLiftDrag,
    "gz::sim::systems::SailLiftDrag")
