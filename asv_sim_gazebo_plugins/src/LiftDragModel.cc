// Copyright (C) 2019  Rhys Mainwaring
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


#include "asv_sim_gazebo_plugins/LiftDragModel.hh"

#include <string>

#include <gazebo/common/Exception.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "asv_sim_gazebo_plugins/Utilities.hh"


namespace asv
{
class LiftDragModelPrivate
{
  /// \brief Fluid density
  public: double fluidDensity = 1.2;

  /// \brief True if the foil is symmetric about its chord.
  public: bool radialSymmetry = true;

  /// \brief Foil forward direction (body frame), usually parallel
  /// to the foil chord.
  public: ignition::math::Vector3d forward =
      ignition::math::Vector3d(1, 0, 0);

  /// \brief Foil upward direction (body frame), usually perpendicular
  /// to the foil chord in the direction of positive lift for the foil
  /// in its intended configuration.
  public: ignition::math::Vector3d upward =
      ignition::math::Vector3d(0, 0, 1);

  /// \brief Foil area
  public: double area = 1.0;

  /// \brief Angle of attack at zero lift.
  public: double alpha0 = 0.0;

  /// \brief Slope of lift coefficient before stall.
  public: double cla = 2.0 * M_PI;

  /// \brief Angle of attack at stall.
  public: double alphaStall = 1.0/2.0/M_PI;

  /// \brief Slope of lift coefficient after stall.
  public: double claStall = -(2*M_PI)/(M_PI*M_PI - 1.0);

  /// \brief Slope of drag coefficient.
  public: double cda = 2.0/M_PI;
};

///////////////////////////////////////////////////////////////////////////////
// LiftDragModel

LiftDragModel::~LiftDragModel()
{
}

LiftDragModel::LiftDragModel() :
  data(new LiftDragModelPrivate())
{
}

LiftDragModel::LiftDragModel(std::unique_ptr<LiftDragModelPrivate>& data)
{
  this->data = std::move(data);
}

LiftDragModel* LiftDragModel::Create(const sdf::ElementPtr& _sdf)
{
  std::unique_ptr<LiftDragModelPrivate> data(new LiftDragModelPrivate());

  // Parameters
  asv::LoadParam(_sdf, "fluid_density", data->fluidDensity,
      data->fluidDensity);
  asv::LoadParam(_sdf, "radial_symmetry", data->radialSymmetry,
      data->radialSymmetry);
  asv::LoadParam(_sdf, "forward", data->forward, data->forward);
  asv::LoadParam(_sdf, "upward", data->upward, data->upward);
  asv::LoadParam(_sdf, "area", data->area, data->area);
  asv::LoadParam(_sdf, "a0", data->alpha0, data->alpha0);
  asv::LoadParam(_sdf, "alpha_stall", data->alphaStall, data->alphaStall);
  asv::LoadParam(_sdf, "cla", data->cla, data->cla);
  asv::LoadParam(_sdf, "cla_stall", data->claStall, data->claStall);
  asv::LoadParam(_sdf, "cda", data->cda, data->cda);

  // Only support radially symmetric lift-drag coefficients at present
  if (!data->radialSymmetry) {
    gzthrow("LiftDragModel only supports radially symmetric foils");
    return 0;
  }

  // Normalise
  data->forward.Normalize();
  data->upward.Normalize();

  return new LiftDragModel(data);
}

void LiftDragModel::Compute(
  const ignition::math::Vector3d& _velU,
  const ignition::math::Pose3d& _bodyPose,
  ignition::math::Vector3d& _lift,
  ignition::math::Vector3d& _drag) const
{
  double alpha, u, cl, cd;
  this->Compute(_velU, _bodyPose, _lift, _drag, alpha, u, cl, cd);
}

void LiftDragModel::Compute(
  const ignition::math::Vector3d& _velU,
  const ignition::math::Pose3d& _bodyPose,
  ignition::math::Vector3d& _lift,
  ignition::math::Vector3d& _drag,
  double& _alpha,
  double& _u,
  double& _cl,
  double& _cd) const
{
  // Unit free stream velocity (world frame).
  auto velUnit = _velU;
  velUnit.Normalize();

  // Avoid division by zero issues.
  if (_velU.Length() <= 0.01)
    return;

  // Rotate forward and upward vectors into the world frame.
  auto forwardI = _bodyPose.Rot().RotateVector(this->data->forward);
  auto upwardI = _bodyPose.Rot().RotateVector(this->data->upward);

  // The span vector is normal to lift-drag-plane (world frame)
  auto spanI = forwardI.Cross(upwardI).Normalize();

  // Compute the angle of attack, alpha:
  // This is the angle between the free stream velocity
  // projected into the lift-drag plane and the forward vector
  auto velLD = _velU - _velU.Dot(spanI) * velUnit;

  // Get direction of drag
  auto dragUnit = velLD;
  dragUnit.Normalize();

  // Get direction of lift
  auto liftUnit = dragUnit.Cross(spanI);
  liftUnit.Normalize();

  // Compute angle of attack.
  double sgnAlpha =  forwardI.Dot(liftUnit) < 0 ? -1.0 : 1.0;
  double cosAlpha = -forwardI.Dot(dragUnit);
  // lift-drag coefficients assume alpha > 0 if foil is symmetric
  double alpha = acos(cosAlpha);

  // Compute dynamic pressure.
  double u = velLD.Length();
  double q = 0.5 * this->data->fluidDensity * u * u;

  // Compute lift coefficient and set sign.
  double cl = this->LiftCoefficient(alpha) * sgnAlpha;

  // Compute lift force.
  _lift = cl * q * this->data->area * liftUnit;

  // Compute drag coefficient.
  double cd = this->DragCoefficient(alpha);

  // Compute drag force.
  _drag = cd * q * this->data->area * dragUnit;

  // Outputs
  _alpha = alpha;
  _u = u;
  _cl = cl;
  _cd = cd;

  // DEBUG
#if 0
  gzmsg << "velU:         " << _velU << "\n";
  gzmsg << "velUnit:      " << velUnit << "\n";
  gzmsg << "body_pos:     " << _bodyPose.Pos() << "\n";
  gzmsg << "body_rot:     " << _bodyPose.Rot().Euler() << "\n";
  gzmsg << "forward:      " << this->data->forward << "\n";
  gzmsg << "upward:       " << this->data->upward << "\n";
  gzmsg << "forwardI:     " << forwardI << "\n";
  gzmsg << "upwardI:      " << upwardI << "\n";
  gzmsg << "spanI:        " << spanI << "\n";
  gzmsg << "velLD:        " << velLD << "\n";
  gzmsg << "dragUnit:     " << dragUnit << "\n";
  gzmsg << "liftUnit:     " << liftUnit << "\n";
  gzmsg << "alpha:        " << alpha << "\n";
  gzmsg << "lift:         " << _lift << "\n";
  gzmsg << "drag:         " << _drag << "\n";
#endif
}

/// Lift is piecewise linear and symmetric about alpha = PI/2
double LiftDragModel::LiftCoefficient(double _alpha) const
{
  double alpha0     = this->data->alpha0;
  double cla        = this->data->cla;
  double alphaStall = this->data->alphaStall;
  double claStall   = this->data->claStall;

  auto f1 = [=](auto _x)
  {
    return cla * (_x - alpha0);
  };

  auto f2 = [=](auto _x)
  {
    if (_x < alphaStall)
      return f1(_x);
    else
      return claStall * (_x - alphaStall) + f1(alphaStall);
  };

  double cl = 0.0;
  if (_alpha < M_PI/2.0)
  {
    cl = f2(_alpha);
  }
  else
  {
    cl = -f2(M_PI - _alpha);
  }

  return cl;
}

/// Drag is piecewise linear and symmetric about alpha = PI/2
double LiftDragModel::DragCoefficient(double _alpha) const
{
  double cda = this->data->cda;

  auto f1 = [=](auto _x)
  {
    return cda * _x;
  };

  double cd = 0.0;
  if (_alpha < M_PI/2.0)
  {
    cd = f1(_alpha);
  }
  else
  {
    cd = f1(M_PI - _alpha);
  }

  return cd;
}

}  // namespace asv
