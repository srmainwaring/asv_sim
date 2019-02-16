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

// Code modified from OSRF LiftDragPlugin

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
#ifndef GAZEBO_PLUGINS_SAIL_LIFT_DRAG_PLUGIN_HH_
#define GAZEBO_PLUGINS_SAIL_LIFT_DRAG_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"

#include <memory>

namespace gazebo
{
  class SailLiftDragPluginPrivate;

  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE SailLiftDragPlugin : public ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~SailLiftDragPlugin();

    /// \brief Constructor.
    public: SailLiftDragPlugin();

    // Documentation Inherited.
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation Inherited.
    public: void Reset() override;

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();
 
    /// \brief Calculate the lift coefficient as a function
    /// of the angle of attack.
    /// \param[in] _alpha Angle of attack in radians.
    public: double LiftCoefficient(double _alpha) const;

    /// \brief Calculate the drag coefficient as a function
    /// of the angle of attack.
    /// \param[in] _alpha Angle of attack in radians.
    public: double DragCoefficient(double _alpha) const;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<SailLiftDragPluginPrivate> data;
  };
}

#endif // GAZEBO_PLUGINS_SAIL_LIFT_DRAG_PLUGIN_HH_
