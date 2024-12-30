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

#include <memory>
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

#include "asv/sim/LiftDragModel.hh"

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
  public: Link link{kNullEntity};

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Publish to topic "/model/<model>/link/<link>/sail_lift_drag".
  public: transport::Node::Publisher liftDragPub;

  /// \brief Update period calculated from <update_rate>.
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Previous update time for publisher throttle.
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief Center of pressure in link local coordinates.
  public: gz::math::Vector3d cpLink = gz::math::Vector3d::Zero;

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
    EventManager &/*_eventMgr*/)
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
    this->dataPtr->cpLink = _sdf->Get<gz::math::Vector3d>("cp");
  }

  {
    double rate = 1.0;
    std::chrono::duration<double> period{rate > 0.0 ? 1.0 / rate : 0.0};
    this->dataPtr->updatePeriod = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period);
  }

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

  if (!this->dataPtr->link.Valid(_ecm) || !this->dataPtr->liftDrag)
    return;

  // ensure components are available
  this->dataPtr->link.EnableVelocityChecks(_ecm, true);
  this->dataPtr->link.EnableAccelerationChecks(_ecm, true);

  /// \todo(srmainwaring) get wind model accounting for wind effects plugin
  // wind velocity
  auto velWindWorld = math::Vector3d::Zero;
  Entity windEntity = _ecm.EntityByComponents(components::Wind());
  auto velWindWorldComp =
      _ecm.Component<components::WorldLinearVelocity>(windEntity);
  if (velWindWorldComp)
  {
    velWindWorld = velWindWorldComp->Data();
  }

  // Pose of link origin and link CoM (world frame).
  auto linkPoseWorldOpt = this->dataPtr->link.WorldPose(_ecm);
  if (!linkPoseWorldOpt.has_value())
    return;
  auto linkPoseWorld = linkPoseWorldOpt.value();

  // Linear velocity at the centre of pressure (world frame).
  auto velCpWorldComp = this->dataPtr->link.WorldLinearVelocity(
      _ecm, this->dataPtr->cpLink);
  if (!velCpWorldComp.has_value())
    return;
  auto velCpWorld = velCpWorldComp.value();

  // Free stream velocity at centre of pressure (world frame).
  auto velWorld = velWindWorld - velCpWorld;

  // Compute lift and drag in world frame
  double alpha = 0;
  double u = 0;
  double cl = 0;
  double cd = 0;
  gz::math::Vector3d lift = gz::math::Vector3d::Zero;
  gz::math::Vector3d drag = gz::math::Vector3d::Zero;
  this->dataPtr->liftDrag->Compute(velWorld, linkPoseWorld,
      lift, drag, alpha, u, cl, cd);

  // Rotate the centre of pressure (CP) into the world frame.
  auto xr = linkPoseWorld.Rot().RotateVector(this->dataPtr->cpLink);

  // Ensure no overflow.
  lift.Correct();
  drag.Correct();

  // Compute torque (about link origin in world frame)
  auto liftTorque = xr.Cross(lift);
  auto dragTorque = xr.Cross(drag);

#if 0
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed >= this->dataPtr->updatePeriod)
  {
    this->dataPtr->lastUpdateTime = _info.simTime;

    // Resultant force arising from lift and drag (world frame).
    auto force = lift + drag;
    auto torque = liftTorque + dragTorque;
    force.Correct();
    torque.Correct();

    gzmsg << "Link:         " << this->dataPtr->link.Name(_ecm).value() << "\n"
          // Scalars
          << "u:            " << u << "\n"
          << "alpha:        " << alpha << "\n"
          << "cl:           " << cl << "\n"
          << "cd:           " << cd << "\n"
          << "||velWind||:  " << velWindWorld.Length() << "\n"
          << "||velCp||:    " << velCpWorld.Length() << "\n"
          << "||vel||:      " << velWorld.Length() << "\n"
          << "||lift||:     " << lift.Length() << "\n"
          << "||drag||:     " << drag.Length() << "\n"
          << "||xr||:       " << xr.Length() << "\n"
          << "||force||:    " << force.Length() << "\n"
          // Vectors
          << "link.pos:     " << linkPoseWorld.Pos() << "\n"
          << "link.rot:     " << linkPoseWorld.Rot().Euler() << "\n"
          << "velWind:      " << velWindWorld << "\n"
          << "velCp:        " << velCpWorld << "\n"
          << "vel:          " << velWorld << "\n"
          << "lift:         " << lift << "\n"
          << "drag:         " << drag << "\n"
          << "liftTorque:   " << liftTorque << "\n"
          << "dragTorque:   " << dragTorque << "\n"
          << "xr:           " << xr << "\n"
          << "force:        " << force << "\n"
          << "torque:       " << torque << "\n"
          << "link          " << linkPoseWorld.Pos() << "\n"
          << "cp:           " << this->dataPtr->cpLink << "\n"
          << "\n";
  }
#endif

  // Add force and torque to link (applied at link origin in world frame).
  if (lift.IsFinite() && liftTorque.IsFinite())
  {
    auto link = Link(this->dataPtr->link.Entity());
    // link.SetVisualizationLabel("SailLift");
    link.AddWorldWrench(_ecm, lift, liftTorque);
  }
  else
  {
    gzwarn << "SailLiftDrag: overflow in lift calculation.\n"
           << "Link:         " << this->dataPtr->link.Name(_ecm).value() << "\n"
           << "xr:           " << xr << "\n"
           << "lift:         " << lift << "\n"
           << "liftTorque:   " << liftTorque << "\n"
           << "\n";
 }
  if (drag.IsFinite() && dragTorque.IsFinite())
  {
    auto link = Link(this->dataPtr->link.Entity());
    // link.SetVisualizationLabel("SailDrag");
    link.AddWorldWrench(_ecm, drag, dragTorque);
  }
  else
  {
    gzwarn << "SailLiftDrag: overflow in drag calculation.\n"
           << "Link:         " << this->dataPtr->link.Name(_ecm).value() << "\n"
           << "xr:           " << xr << "\n"
           << "drag:         " << drag << "\n"
           << "dragTorque:   " << dragTorque << "\n"
           << "\n";
  }
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
