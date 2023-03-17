// Copyright (C) 2022-2023  Rhys Mainwaring
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

#include "WindControl.hh"

#include <gz/msgs/any.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/param_v.pb.h>

#include <gz/transport/Node.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>

#include <mutex>
#include <string>

namespace gz
{
namespace sim
{
inline namespace ASV_SIM_VERSION_NAMESPACE
{
  /// \brief Private data class for WindControl
  class WindControlPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Wind speed
    public: double windSpeed{5.0};

    /// \brief Wind angle
    public: double windAngle{135.0};

    /// \brief Mutex windSpeed and windAngle
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Name of the world
    public: std::string worldName;
  };
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
WindControl::WindControl()
  : GuiSystem(), dataPtr(new WindControlPrivate)
{
}

/////////////////////////////////////////////////
WindControl::~WindControl() = default;

/////////////////////////////////////////////////
void WindControl::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
  {
    this->title = "Wind Control";
  }
}

//////////////////////////////////////////////////
void WindControl::Update(const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    // Get the name of the world
    if (this->dataPtr->worldName.empty())
    {
      _ecm.Each<components::World, components::Name>(
        [&](const Entity &,
            const components::World *,
            const components::Name *_name) -> bool
        {
          // Assume there's only one world
          this->dataPtr->worldName = _name->Data();
          return false;
        });
    }

    this->dataPtr->initialized = true;
  }
}

//////////////////////////////////////////////////
void WindControl::UpdateWindSpeed(double _windSpeed)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windSpeed = _windSpeed;

  gzmsg << "Wind Speed: " << _windSpeed << "\n";
}

//////////////////////////////////////////////////
void WindControl::UpdateWindAngle(double _windAngle)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windAngle = _windAngle;

  gzmsg << "Wind Angle: " << _windAngle << "\n";
}

}  // namespace sim
}  // namespace gz

//////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(gz::sim::WindControl,
              gz::gui::Plugin)
