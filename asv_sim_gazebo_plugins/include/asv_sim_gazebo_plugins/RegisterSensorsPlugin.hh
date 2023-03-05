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

#ifndef ASV_SIM_GAZEBO_PLUGINS_REGISTERSENSORSPLUGIN_HH_
#define ASV_SIM_GAZEBO_PLUGINS_REGISTERSENSORSPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>

namespace asv
{
/// \brief A Gazebo system plugin to register custom sensors and messages.
///
/// # Usage
///
/// From the command line:
///
/// \code
/// gzserver --server-plugin libRegisterSensorsPlugin.so
/// \endcode
///
/// \code
/// gzclient --gui-plugin libRegisterSensorsPlugin.so
/// \endcode
///
/// In a roslaunch file:
///
/// \code
/// <arg name="extra_gazebo_args"
///   default="--server-plugin libRegisterSensorsPlugin.so
///            --gui-plugin libRegisterSensorsPlugin.so"
/// />
/// \endcode
///
/// Note:
/// To display custom message types in 'Gazebo: Text View' the parameter
/// `extra_gazebo_args` must be passed to the gazebo client. The default
/// `empty_world.launch` file in `gazebo_ros_pkgs/gazebo_ros` does not do this,
/// so either create your own copy, or if building `gazebo_ros` from source,
/// edit `gazebo_ros/launch/empty_world.launch` and modify:
///
/// \code
/// <!-- start gazebo client -->
/// <group if="$(arg gui)">
///   <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false"
///      output="screen" args="$(arg command_arg3)"/>
/// </group>
/// \endcode
///
/// to:
///
/// \code
/// <!-- start gazebo client -->
/// <group if="$(arg gui)">
///   <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false"
///     output="screen" args="$(arg command_arg3) $(arg extra_gazebo_args)"/>
/// </group>
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
}  // namespace asv

#endif  // ASV_SIM_GAZEBO_PLUGINS_REGISTERSENSORSPLUGIN_HH_
