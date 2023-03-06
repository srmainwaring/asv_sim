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

// This code modified from the gazebo LiftDrag plugin
/*
* Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef ASV_SIM_LIFTDRAGMODEL_HH_
#define ASV_SIM_LIFTDRAGMODEL_HH_

#include <memory>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <sdf/sdf.hh>

namespace asv
{
class LiftDragModelPrivate;

/// \brief A class to calculate lift / drag.
class LiftDragModel
{
  /// \brief Destructor.
  public: virtual ~LiftDragModel();

  /// \brief Constructor.
  public: LiftDragModel();

  /// \brief Create a new LiftDragModel from SDF.
  /// \param[in] _sdf A pointer to an SDF element containing parameters.
  public: static LiftDragModel* Create(
      const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Compute the lift and drag forces in the world frame.
  /// param[in] _velU     Free-stream velocity vector.
  /// param[out] _lift    Lift vector.
  /// param[out] _drag    Drag vector.
  public: void Compute(
    const gz::math::Vector3d &_velU,
    const gz::math::Pose3d &_bodyPose,
    gz::math::Vector3d &_lift,
    gz::math::Vector3d &_drag) const;

  /// \brief Compute the lift and drag forces in the world frame.
  /// param[in] _velU     Free-stream velocity vector.
  /// param[out] _lift    Lift vector.
  /// param[out] _drag    Drag vector.
  /// param[out] _alpha   Angle of attack in radians.
  /// param[out] _u       Free-stream speed in the lift/drag plane.
  /// param[out] _cd      Lift coefficient.
  /// param[out] _cd      Drag coefficient.
  public: void Compute(
    const gz::math::Vector3d &_velU,
    const gz::math::Pose3d &_bodyPose,
    gz::math::Vector3d &_lift,
    gz::math::Vector3d &_drag,
    double &_alpha,
    double &_u,
    double &_cl,
    double &_cd) const;

  /// \brief The lift coefficient as a function of the angle of attack.
  /// \param[in] _alpha Angle of attack in radians.
  public: double LiftCoefficient(double _alpha) const;

  /// \brief The drag coefficient as a function of the angle of attack.
  /// \param[in] _alpha Angle of attack in radians.
  public: double DragCoefficient(double _alpha) const;

  /// \internal
  /// \brief Constructor, ownership transferred from data.
  private: LiftDragModel(std::unique_ptr<LiftDragModelPrivate> &_data);

  /// \internal
  /// \brief Pointer to the class private data.
  private: std::unique_ptr<LiftDragModelPrivate> data;
};

}  // namespace asv

#endif  // ASV_SIM_LIFTDRAGMODEL_HH_
