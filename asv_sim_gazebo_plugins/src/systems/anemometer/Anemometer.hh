// Copyright (C) 2019-2023 Rhys Mainwaring
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

/// \file Anemometer.hh
/// \brief This file defines a sensor used to measure wind speed and direction.

#ifndef GZ_SIM_SYSTEMS_ANEMOMETER_HH_
#define GZ_SIM_SYSTEMS_ANEMOMETER_HH_

#include <memory>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

// Forward declarations.
class AnemometerPrivate;

/// \brief Anemometer to measure wind speed and direction.
///
/// #Usage
///
/// Add the SDF for the sensor to a <link> element of your model.
///
/// \code
/// <sensor name="anemometer_sensor" type="anemometer">
///   <always_on>true</always_on>
///   <update_rate>50</update_rate>
///   <topic>anemometer</topic>
/// </sensor>
/// \endcode
///
/// # Published Topics
///
/// 1. ~/anemometer (asv_msgs::msgs::Anemometer)
///   - time (gazebo::msgs::Time)
///     The simulation time of the observation.
///   - wind_velocity (gazebo::msgs::Vector3d)
///     The apparent wind at the sensor origin.
///     (i.e. true wind adjusted for the velocity of the sensor).
///
/// # Parameters
///
/// 1. <always_on> (bool, default: false)
///   Standard <sensor> parameter. See SDF documentation for details.
///
/// 2. <update_rate> (double, default: 0)
///   Standard <sensor> parameter. See SDF documentation for details.
///
/// 3. <topic> (string, default: ~/anemometer)
///   Standard <sensor> parameter. See SDF documentation for details.
///
class Anemometer
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate
{
  /// \brief Destructor.
  public: ~Anemometer() override;

  /// \brief Constructor.
  public: Anemometer();

  // Documentation inherited
  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) final;

  /// Documentation inherited
  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  /// Documentation inherited
  public: void PostUpdate(
      const UpdateInfo &_info,
      const EntityComponentManager &_ecm) final;

  /// \brief Accessor for the current true wind velocity in m/s.
  /// \return Current true wind velocity.
  public: gz::math::Vector3d TrueWindVelocity() const;

  /// \brief Accessor for the current apparent wind velocity in m/s.
  /// \return Current apparent wind velocity.
  public: gz::math::Vector3d ApparentWindVelocity() const;

  /// \brief Private data pointer.
  private: std::unique_ptr<AnemometerPrivate> dataPtr;
};

/// \def AnemometerPtr
/// \brief Shared pointer to Anemometer
using AnemometerPtr = std::shared_ptr<Anemometer>;

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_ANEMOMETER_HH_