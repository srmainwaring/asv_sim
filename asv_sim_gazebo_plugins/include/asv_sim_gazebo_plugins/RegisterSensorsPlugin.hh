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

/// \file RegisterSensorsPlugin.hh
/// \brief A system plugin to register custom sensors.

#ifndef _ASV_SIM_GAZEBO_PLUGINS_REGISTER_SENSORS_PLUGIN_HH_
#define _ASV_SIM_GAZEBO_PLUGINS_REGISTER_SENSORS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

namespace asv
{
  /// \brief A Gazebo system plugin to register custom sensors.
  ///
  /// # Usage
  ///
  /// From the command line:
  ///
  /// \code
  /// gzserver --server-plugin libRegisterSensorsPlugin.so
  /// \endcode
  ///
  /// In a roslaunch file:
  ///
  /// \code
  /// <arg
  ///   name="extra_gazebo_args"
  ///   default="--server-plugin libRegisterSensorsPlugin.so"
  /// />
  /// \endcode
  ///
  class GAZEBO_VISIBLE RegisterSensorsPlugin : public gazebo::SystemPlugin
  {
    /// \brief Destructor.
    public: virtual ~RegisterSensorsPlugin() override;

    /// \brief Constructor.
    public: RegisterSensorsPlugin();

    // Documentation inherited.
    public: void Load(int _argc, char** _argv) override;

    // Documentation inherited.
    public: void Init() override;
  };
} // namespace asv

#endif // _ASV_SIM_GAZEBO_PLUGINS_REGISTER_SENSORS_PLUGIN_HH_
