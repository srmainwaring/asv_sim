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
#ifndef _ASV_SIM_GAZEBO_PLUGINS_SAIL_PLUGIN_HH_
#define _ASV_SIM_GAZEBO_PLUGINS_SAIL_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"

#include <memory>

namespace asv
{
  class SailPluginPrivate;

  /// \brief A plugin that simulates lift and drag on a sail.
  class GAZEBO_VISIBLE SailPlugin : public gazebo::ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~SailPlugin();

    /// \brief Constructor.
    public: SailPlugin();

    // Documentation Inherited.
    public: void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation Inherited.
    public: void Reset() override;

    /// \brief Create a topic string for the sensor.
    public: virtual std::string GetTopic() const;

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();
 
    /// \internal
    /// \brief Pointer to the class private data.
    private: std::unique_ptr<SailPluginPrivate> data;
  };

} // namespace asv

#endif // _ASV_SIM_GAZEBO_PLUGINS_SAIL_PLUGIN_HH_
