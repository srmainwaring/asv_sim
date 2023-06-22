// Copyright (C) 2023 Rhys Mainwaring
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

// Adapted from the MooringForce system developed for the MBARI wave buoy
// including https://github.com/osrf/buoy_sim/pull/135
//
// Authors: Mabel Zhang <mabel@openrobotics.org>
//          Michael Anderson <anderson@mbari.org>
//          Rhys Mainwaring <rhys.mainwaring@me.com>

// Copyright 2022 Open Source Robotics Foundation, Inc.
//                and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Winch.hh"

#include <cmath>
#include <memory>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>

namespace gz
{
namespace sim
{
namespace systems
{
/////////////////////////////////////////////////
class WinchPrivate
{
  /// \brief Model interface.
  public: sim::Model model{sim::kNullEntity};

  /// \brief The link attached to the winch cable.
  public: sim::Link link{sim::kNullEntity};

  /// \brief World position of link.
  public: math::Vector3d linkWorldPos;

  /// \brief Name of the link the cable is attached to.
  public: std::string linkName;

  /// \brief The fixed position of the winch in the world.
  public: math::Vector3d winchWorldPos;

  /// \brief Meters, vertical distance from tethered object to winch.
  ///  Updated per iteration.
  public: double V{std::nanf("")};

  /// \brief Meters, total length of cable.
  public: double L{std::nanf("")};

  /// \brief Meters, horizontal distance from tethered object to winch.
  /// Updated per iteration.
  public: double H{std::nanf("")};

  /// \brief Mass of cable per unit length (kg/m)
  public: double cableMassPerMetre{std::nanf("")};

  /// \brief Weight of cable per unit length (N/m).
  public: double w{std::nanf("")};

  /// \brief radians, atan2 angle of object from winch.
  public: double theta{std::nanf("")};

  /// \brief Catenary equation to pass to solver.
  public: std::unique_ptr<CatenaryHSoln> catenarySoln;

  /// \brief Solution to catenary equation. Meters, length of cable laying on
  /// the surface, start of catenary.
  public: Eigen::VectorXd B{};

  /// \brief Debug print period calculated from <debug_print_rate>
  public: std::chrono::steady_clock::duration debugPrintPeriod{0};

  /// \brief Last debug print simulation time
  public: std::chrono::steady_clock::duration lastDebugPrintTime{0};

  /// \brief Constructor
  public: WinchPrivate();

  /// \brief Look for tethered object link to find input to catenary equation,
  ///  and link to apply output force to.
  public: bool FindLinks(sim::EntityComponentManager &_ecm);

  /// \brief Update V and H for solver input.
  public: void UpdateVH(sim::EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
WinchPrivate::WinchPrivate()
    : catenarySoln(std::make_unique<CatenaryHSoln>(V, H, L))
{
}

//////////////////////////////////////////////////
bool WinchPrivate::FindLinks(sim::EntityComponentManager &_ecm)
{
  // Find link
  auto entity = this->model.LinkByName(_ecm, this->linkName);
  this->link = sim::Link(entity);

  if (!this->link.Valid(_ecm))
  {
    gzerr << "[Winch] could not find link[" << this->linkName << "]. "
          << "Winch force will not be calculated."
          << std::endl;
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void WinchPrivate::UpdateVH(sim::EntityComponentManager &_ecm)
{
  // Skip if buoy link is not valid.
  if (!this->link.Valid(_ecm))
  {
    return;
  }

  // Get tethered object position in world.
  auto linkWorldPose = this->link.WorldPose(_ecm);
  this->linkWorldPos = linkWorldPose->Pos();

  // Update vertical (z) distance between object and winch.
  this->V = std::fabs(this->linkWorldPos[2U] - this->winchWorldPos[2U]);

  // Update horizontal distance between object and winch.
  this->H = std::sqrt(
    (this->linkWorldPos[0U] - this->winchWorldPos[0U]) *
    (this->linkWorldPos[0U] - this->winchWorldPos[0U]) +
    (this->linkWorldPos[1U] - this->winchWorldPos[1U]) *
    (this->linkWorldPos[1U] - this->winchWorldPos[1U]));

  // Update angle between object and winch.
  this->theta = std::atan2(this->linkWorldPos[1U] - this->winchWorldPos[1U],
      this->linkWorldPos[0U] - this->winchWorldPos[0U]);

  this->catenarySoln.reset(new CatenaryHSoln(this->V, this->H, this->L));
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
Winch::~Winch() = default;

/////////////////////////////////////////////////
Winch::Winch()
  : System(), dataPtr(std::make_unique<WinchPrivate>())
{
}

/////////////////////////////////////////////////
void Winch::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Winch plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // Required parameters
  if (_sdf->HasElement("link_name"))
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
    gzdbg << "Link name set to " << this->dataPtr->linkName
          << std::endl;
  }
  else
  {
    gzerr << "[Winch] missing required element <link_name>. "
          << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("winch_position"))
  {
    this->dataPtr->winchWorldPos = _sdf->Get<math::Vector3d>(
        "winch_position");
    gzdbg << "Winch position set to " << this->dataPtr->winchWorldPos
          << std::endl;
  }
  else
  {
    gzerr << "[Winch] missing required element <winch_position>. "
          << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("cable_length"))
  {
    this->dataPtr->L = _sdf->Get<double>("cable_length");
    gzdbg << "Cable length set to " << this->dataPtr->L << std::endl;
  }
  else
  {
    gzerr << "[Winch] missing required element <cable_length>. "
          << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("cable_mass_per_metre"))
  {
    this->dataPtr->cableMassPerMetre =
        _sdf->Get<double>("cable_mass_per_metre");
    gzdbg << "Cable mass per length set to "
          << this->dataPtr->cableMassPerMetre << std::endl;

    /// \todo(srmainwaring) move to constants.
    double gravity = 9.81;
    this->dataPtr->w = gravity * this->dataPtr->cableMassPerMetre;
  }
  else
  {
    gzerr << "[Winch] missing required element <cable_mass_per_metre>. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // Optional parameters

  // debug print throttle, default 1Hz
  {
    double rate(1.0);
    if (_sdf->HasElement("debug_print_rate"))
    {
      rate = _sdf->Get<double>("debug_print_rate", rate).first;
      gzdbg << "Debug print rate set to " << rate << std::endl;
    }
    std::chrono::duration<double> period{rate > 0.0 ? 1.0 / rate : 0.0};
    this->dataPtr->debugPrintPeriod = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period);
  }

  // Find necessary model links
  if (this->dataPtr->FindLinks(_ecm))
  {
    this->dataPtr->UpdateVH(_ecm);
  }

  this->dataPtr->B.resize(1U);
}

/////////////////////////////////////////////////
void Winch::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Winch::PreUpdate");

  // Skip if buoy link is not valid.
  if (!this->dataPtr->link.Valid(_ecm))
  {
    return;
  }

  /// \todo(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Skip if paused.
  if (_info.paused)
  {
    return;
  }

  // Update V and H based on latest buoy position
  this->dataPtr->UpdateVH(_ecm);

  // Skip solver if the  cable can drop vertically (within tolerance).
  double toleranceL = 0.1;
  if (this->dataPtr->V + this->dataPtr->H <= this->dataPtr->L + toleranceL)
  {
    // Assume all force is vertical.
    double Tz = - this->dataPtr->w * this->dataPtr->V;

    math::Vector3d force(0.0, 0.0, Tz);
    math::Vector3d torque = math::Vector3d::Zero;
    if (force.IsFinite() && torque.IsFinite())
    {
      // this->dataPtr->link.SetVisualizationLabel("Winch");
      this->dataPtr->link.AddWorldWrench(_ecm, force, torque);
    }
    else
    {
      gzerr << "[Winch] force is not finite.\n";
    }
    return;
  }

  Eigen::HybridNonLinearSolver<CatenaryHSoln> catenarySolver(
    *this->dataPtr->catenarySoln);
  // Tolerance for error between two consecutive iterations
  catenarySolver.parameters.xtol = 0.001;
  // Max number of calls to the function
  catenarySolver.parameters.maxfev = 20;
  catenarySolver.diag.setConstant(1, 1.0);
  // Improves solution stability dramatically.
  catenarySolver.useExternalScaling = true;

  // Initial estimate for B (upper bound).
  auto BMax = [](double V, double H, double L) -> double
  {
    return (L * L - (V * V + H * H)) / (2 * (L - H));
  };

  double bMax = BMax(this->dataPtr->V, this->dataPtr->H, this->dataPtr->L);

  this->dataPtr->B[0] = bMax;
  int solverInfo = catenarySolver.solveNumericalDiff(this->dataPtr->B);

  double c = CatenaryFunction::CatenaryScalingFactor(
    this->dataPtr->V, this->dataPtr->B[0U], this->dataPtr->L);

  // Horizontal component of cable tension, in Newtons
  // Force at buoy heave cone is Fx = -Tx
  double Tr = - c * this->dataPtr->w;
  double Tx = Tr * std::cos(this->dataPtr->theta);
  double Ty = Tr * std::sin(this->dataPtr->theta);
  // Vertical component of cable tension at attachment point, in Newtons.
  double Tz = - this->dataPtr->w * (this->dataPtr->L - this->dataPtr->B[0U]);

  #if 0
  // Throttle update rate
  auto elapsed = _info.simTime - this->dataPtr->lastDebugPrintTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed >= this->dataPtr->debugPrintPeriod)
  {
    this->dataPtr->lastDebugPrintTime = _info.simTime;
    gzdbg << "HSolver solverInfo: " << solverInfo << "\n"
          << " t: " << std::chrono::duration_cast<
              std::chrono::milliseconds>(_info.simTime).count()/1000.0 << "\n"
          << " Anchor: " << this->dataPtr->winchWorldPos << "\n"
          << " Link:   " << this->dataPtr->linkWorldPos << "\n"
          << " R: " << this->dataPtr->effectiveRadius << "\n"
          << " L: " << this->dataPtr->L << "\n"
          << " V + H : " << this->dataPtr->V + this->dataPtr->H << "\n"
          << " V: " << this->dataPtr->V << "\n"
          << " H: " << this->dataPtr->H << "\n"
          << " b: " << bMax << "\n"
          << " B: " << this->dataPtr->B[0U] << "\n"
          << " c: " << c << "\n"
          << " theta: " << this->dataPtr->theta << "\n"
          << " Tx: " << Tx << "\n"
          << " Ty: " << Ty << "\n"
          << " Tr: " << Tr << "\n"
          << " Tz: " << Tz << "\n"
          << " nfev: " << catenarySolver.nfev << "\n"
          << " iter: " << catenarySolver.iter << "\n"
          << " fnorm: " << catenarySolver.fnorm << "\n"
          << "\n";
  }
  #endif

  // Did not find solution.
  if (solverInfo != 1)
  {
    gzerr << "[Winch] solver failed to converge, solverInfo: "
          << solverInfo << "\n";
    return;
  }

  math::Vector3d force(Tx, Ty, Tz);
  math::Vector3d torque = math::Vector3d::Zero;
  if (force.IsFinite() && torque.IsFinite())
  {
    // this->dataPtr->link.SetVisualizationLabel("Winch");
    this->dataPtr->link.AddWorldWrench(_ecm, force, torque);
  }
  else
  {
    gzerr << "[Winch] force is not finite.\n";
  }
}

}  // namespace systems
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::Winch,
    gz::sim::System,
    gz::sim::systems::Winch::ISystemConfigure,
    gz::sim::systems::Winch::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::Winch,
    "gz::sim::systems::Winch")
