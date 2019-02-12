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

/// \file AnemometerSensorPlugin.hh
/// \brief This file defines a Gazebo SensorPlugin used to measure wind speed.

#ifndef _ASV_SIM_GAZEBO_PLUGINS_ANEMOMETER_SENSOR_PLUGIN_HH_
#define _ASV_SIM_GAZEBO_PLUGINS_ANEMOMETER_SENSOR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

#include <memory>

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// AnemometerSensorPluginPlugin

  /// \internal
  /// \brief Class to hold private data for AnemometerSensorPlugin.
  class AnemometerSensorPluginPrivate;

  /// \brief A Gazebo model plugin to simulate water waves.
  ///
  /// # Usage
  /// 
  /// Add the SDF for the plugin to the <model> element of your wave model. 
  ///
  /// \code
  /// <plugin name="anemometer" filename="libAnemometerSensorPlugin.so">
  ///   <static>false</static>
  /// </plugin>
  /// \endcode
  ///
  /// # Published Topics
  ///
  /// 1. ~/wind (gazebo::msgs::Response)
  ///
  /// # Parameters
  ///
  /// 1. <static> (bool, default: false)
  ///   Create a static wave field if set to true.
  ///
  class GAZEBO_VISIBLE AnemometerSensorPlugin : public gazebo::ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~AnemometerSensorPlugin() override;

    /// \brief Constructor.
    public: AnemometerSensorPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited.
    public: void Init() override;

    // Documentation inherited.
    public: void Reset() override;

    /// internal
    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<AnemometerSensorPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_SIM_GAZEBO_PLUGINS_ANEMOMETER_SENSOR_PLUGIN_HH_
