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

    /// \brief Topic
    Q_PROPERTY(
      QString topic
      READ Topic
      WRITE SetTopic
      NOTIFY TopicChanged
    )

    /// \brief WindSpeed
    Q_PROPERTY(
      double windSpeed
      READ WindSpeed
      WRITE SetWindSpeed
      NOTIFY WindSpeedChanged
    )

    /// \brief WindDirection
    Q_PROPERTY(
      double windDirection
      READ WindDirection
      WRITE SetWindDirection
      NOTIFY WindDirectionChanged
    )

    /// \brief Constructor
    public: WindControl();

    /// \brief Destructor
    public: ~WindControl() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    // /// \brief Update the wind topic
    // /// \param[in] _topic new wind topic
    // public slots: void UpdateWindTopic(const std::string &_topic);

    // /// \brief Update the wind speed
    // /// \param[in] _windSpeed new wind speed in m/s
    // public slots: void UpdateWindSpeed(double _windSpeed);

    // /// \brief Update the wind angle
    // /// \param[in] _windAngleDeg new wind angle in degrees
    // public slots: void UpdateWindAngle(double _windAngleDeg);

    /// \brief Get the topic as a string, for example
    /// '/wind'
    /// \return Topic
    public: Q_INVOKABLE QString Topic() const;

    /// \brief Set the topic from a string, for example
    /// '/wind'
    /// \param[in] _topic Topic
    public: Q_INVOKABLE void SetTopic(const QString &_topic);

    /// \brief Notify that topic has changed
    signals: void TopicChanged();

    /// \brief Get the wind speed, in m/s
    /// \return Wind speed
    public: Q_INVOKABLE double WindSpeed() const;

    /// \brief Set the wind speed, in m/s
    /// \param[in] _windSpeed Wind speed
    public: Q_INVOKABLE void SetWindSpeed(const double _windSpeed);

    /// \brief Notify that wind speed has changed
    signals: void WindSpeedChanged();

    /// \brief Get the wind direction, in degrees from North
    /// \return Wind direction
    public: Q_INVOKABLE double WindDirection() const;

    /// \brief Set the wind direction, in degrees from North
    /// \param[in] _windDirection Wind direction
    public: Q_INVOKABLE void SetWindDirection(const double _windDirection);

    /// \brief Notify that wind direction has changed
    signals: void WindDirectionChanged();


    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<WindControlPrivate> dataPtr;
  };

}
}
}

#endif  // ASV_SIM_WINDCONTROL_HH_
