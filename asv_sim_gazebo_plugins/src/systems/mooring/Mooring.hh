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


#ifndef ASV_SIM_MOORING_HH_
#define ASV_SIM_MOORING_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>

#include "CatenarySoln.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

// Forward declarations.
class MooringPrivate;

/// \brief A plugin that simulates a mooring.
class Mooring
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  /// \brief Destructor.
  public: virtual ~Mooring();

  /// \brief Constructor.
  public: Mooring();

  // Documentation inherited
  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) final;

  /// Documentation inherited
  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  /// \brief Private data pointer.
  private: std::unique_ptr<MooringPrivate> dataPtr;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // ASV_SIM_MOORING_HH_
