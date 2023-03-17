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

#ifndef ASV_SIM_WINDCONTROL_HH_
#define ASV_SIM_WINDCONTROL_HH_

#include <gz/gui/qt.h>
#include <gz/sim/gui/GuiSystem.hh>

#include <memory>

namespace gz
{
namespace sim
{

// Inline bracket to help doxygen filtering.
inline namespace ASV_SIM_VERSION_NAMESPACE
{
  class WindControlPrivate;

  /// \brief Edit the wind environment.
  class WindControl : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: WindControl();

    /// \brief Destructor
    public: ~WindControl() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Update the wind speed
    /// \param[in] _windSpeed new wind speed
    public slots: void UpdateWindSpeed(double _windSpeed);

    /// \brief Update the wind angle
    /// \param[in] _windAngle new wind angle
    public slots: void UpdateWindAngle(double _windAngle);

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<WindControlPrivate> dataPtr;
  };

}
}
}

#endif  // ASV_SIM_WINDCONTROL_HH_
