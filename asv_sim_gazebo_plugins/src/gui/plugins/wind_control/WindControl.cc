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

#include <mutex>
#include <string>

#include <gz/plugin/Register.hh>

#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>

#include <gz/transport/Node.hh>

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
    public: double windAngleDeg{135.0};

    /// \brief Mutex windSpeed and windAngleDeg
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

    // todo - set the GUI with the initial value of the wind


    this->dataPtr->initialized = true;
  }

  /// \todo(srmainwaring) only update on change 

  // set the wind if changed
  Entity windEntity = _ecm.EntityByComponents(components::Wind());

  // /// \todo(srmainwaring) what to do with vz-component
  double windAngleRad = this->dataPtr->windAngleDeg * GZ_PI / 180.0;
  double vx = std::cos(windAngleRad) * this->dataPtr->windSpeed;
  double vy = std::sin(windAngleRad) * this->dataPtr->windSpeed;

  // // update wind velocity
  math::Vector3d windVelocity(vx, vy, 0.0);

  auto windVelComp =
      _ecm.Component<components::WorldLinearVelocity>(windEntity);

  if (windVelComp)
  {
    auto compFunc = [](const math::Vector3d &_a, const math::Vector3d &_b)
    {
      return _a == _b;
    };
    auto state = windVelComp->SetData(windVelocity, compFunc)
        ? ComponentState::PeriodicChange
        : ComponentState::NoChange;
    _ecm.SetChanged(windEntity,
        components::WorldLinearVelocity::typeId,
        ComponentState::OneTimeChange /*state*/);
  }
  else
  {
    _ecm.CreateComponent(windEntity,
        components::WorldLinearVelocity(windVelocity));
  }

  // debug
  {
    auto windVel =
        _ecm.Component<components::WorldLinearVelocity>(windEntity)->Data();
    gzdbg << "wind: " << windVel << "\n";
  }
}

//////////////////////////////////////////////////
void WindControl::UpdateWindSpeed(double _windSpeed)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windSpeed = _windSpeed;

  gzmsg << "Wind Speed: " << _windSpeed << " (m/s)\n";
}

//////////////////////////////////////////////////
void WindControl::UpdateWindAngle(double _windAngleDeg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windAngleDeg = _windAngleDeg;

  gzmsg << "Wind Angle: " << _windAngleDeg << " (deg)\n";
}

}  // namespace sim
}  // namespace gz

//////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(gz::sim::WindControl,
              gz::gui::Plugin)
