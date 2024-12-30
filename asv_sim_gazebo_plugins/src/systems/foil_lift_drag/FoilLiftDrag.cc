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

#include "FoilLiftDrag.hh"

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
class FoilLiftDragPrivate
{
  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Link interface.
  public: Link link{kNullEntity};

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Publish to topic "/model/<model>/link/<link>/foil_lift_drag".
  public: transport::Node::Publisher liftDragPub;

  /// \brief Update period calculated from <update_rate>.
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Previous update time for publisher throttle.
  public: std::chrono::steady_clock::duration prevTime{0};

  /// \brief Center of pressure in link local coordinates.
  public: gz::math::Vector3d cpLink = gz::math::Vector3d::Zero;

  /// \brief Lift drag model.
  public: std::unique_ptr<asv::LiftDragModel> liftDrag;
};

/////////////////////////////////////////////////
FoilLiftDrag::~FoilLiftDrag() = default;

/////////////////////////////////////////////////
FoilLiftDrag::FoilLiftDrag()
  : System(), dataPtr(std::make_unique<FoilLiftDragPrivate>())
{
}

/////////////////////////////////////////////////
void FoilLiftDrag::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "FoilLiftDrag plugin should be attached to a model "
           << "entity. Failed to initialize.\n";
    return;
  }

  // Parameters
  {
    if (!_sdf->HasElement("link_name"))
    {
      gzerr << "You must specify a <link_name> for the FoilLiftDrag"
            << " plugin to act upon.\n";
      return;
    }
    auto linkName = _sdf->Get<std::string>("link_name");
    auto linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
    if (!_ecm.HasEntity(linkEntity))
    {
      gzerr << "Link with name [" << linkName << "] not found "
            << "in model [" << this->dataPtr->model.Name(_ecm) << "]. "
            << "The FoilLiftDrag plugin will not generate forces.\n";
      return;
    }
    this->dataPtr->link = Link(linkEntity);
  }

  {
    if (!_sdf->HasElement("cp"))
    {
      gzerr << "You must specify a <cp> for the FoilLiftDrag"
            << " plugin forces to act at.\n";
      return;
    }
    this->dataPtr->cpLink = _sdf->Get<gz::math::Vector3d>("cp");
  }

  {
    double rate = 50.0;
    std::chrono::duration<double> period{rate > 0.0 ? 1.0 / rate : 0.0};
    this->dataPtr->updatePeriod = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period);
  }

  // Lift / Drag model
  this->dataPtr->liftDrag.reset(asv::LiftDragModel::Create(_sdf));
}

/////////////////////////////////////////////////
void FoilLiftDrag::PreUpdate(
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
  auto velWorld = - velCpWorld;

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

  // Add force and torque to link (applied at link origin in world frame).
  if (lift.IsFinite() && liftTorque.IsFinite())
  {
    auto link = Link(this->dataPtr->link.Entity());
    // link.SetVisualizationLabel("FoilLift");
    link.AddWorldWrench(_ecm, lift, liftTorque);
  }
  else
  {
    gzwarn << "FoilLiftDrag: overflow in lift calculation\n"
           << "Link:         " << this->dataPtr->link.Name(_ecm).value() << "\n"
           << "xr:           " << xr << "\n"
           << "lift:         " << lift << "\n"
           << "liftTorque:   " << liftTorque << "\n"
           << "\n";
  }
  if (drag.IsFinite() && dragTorque.IsFinite())
  {
    auto link = Link(this->dataPtr->link.Entity());
    // link.SetVisualizationLabel("FoilDrag");
    link.AddWorldWrench(_ecm, drag, dragTorque);
  }
  else
  {
    gzwarn << "FoilLiftDrag: overflow in drag calculation\n"
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
    gz::sim::systems::FoilLiftDrag,
    gz::sim::System,
    gz::sim::systems::FoilLiftDrag::ISystemConfigure,
    gz::sim::systems::FoilLiftDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::FoilLiftDrag,
    "gz::sim::systems::FoilLiftDrag")
