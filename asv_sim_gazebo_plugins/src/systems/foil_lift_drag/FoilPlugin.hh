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

#ifndef ASV_SIM_GAZEBO_PLUGINS_FOILPLUGIN_HH_
#define ASV_SIM_GAZEBO_PLUGINS_FOILPLUGIN_HH_

#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>


namespace asv
{
class FoilPluginPrivate;

/// \brief A plugin that simulates lift and drag on a foil moving
/// in a stationary fluid.
class GAZEBO_VISIBLE FoilPlugin : public gazebo::ModelPlugin
{
  /// \brief Destructor.
  public: virtual ~FoilPlugin();

  /// \brief Constructor.
  public: FoilPlugin();

  // Documentation Inherited.
  public: void Load(gazebo::physics::ModelPtr _model,
      sdf::ElementPtr _sdf) override;

  // Documentation Inherited.
  public: void Reset() override;

  /// \brief Create a topic string for the sensor.
  public: virtual std::string GetTopic() const;

  /// \brief Callback for World Update events.
  protected: virtual void OnUpdate();

  /// \internal
  /// \brief Pointer to the class private data.
  private: std::unique_ptr<FoilPluginPrivate> data;
};

}  // namespace asv

#endif  // ASV_SIM_GAZEBO_PLUGINS_FOILPLUGIN_HH_
