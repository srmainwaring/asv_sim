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

#include <cmath>
#include <memory>
#include <string>

#include <gz/math/Vector3.hh>

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/sim/System.hh>

#include <gz/transport/Node.hh>

namespace custom
{
/// \brief Custom anemometer sensor that publishes wind speed and direction.
class Anemometer : public gz::sensors::Sensor
{
  /// \brief Destructor.
  public: virtual ~Anemometer();

  // Documentation inherited
  public: virtual bool Load(const sdf::Sensor &_sdf) override;

  // Documentation inherited
  public: virtual bool Update(
    const std::chrono::steady_clock::duration &_now) override;

  /// \brief Set the apparent wind velocity.
  public: void SetApparentWindVelocity(const gz::math::Vector3d &_vel);

  /// \brief Get the latest apparent wind velocity.
  public: const gz::math::Vector3d& ApparentWindVelocity() const;

  /// \brief Previous apparent wind velocity.
  private: gz::math::Vector3d prevApparentWindVel{std::nan(""), std::nan(""),
      std::nan("")};

  /// \brief Noise that will be applied to the sensor data
  private: gz::sensors::NoisePtr noise{nullptr};

  /// \brief Node for communication
  private: gz::transport::Node node;

  /// \brief Publishes sensor data
  private: gz::transport::Node::Publisher pub;
};
}  // namespace custom

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
/// <sensor name="anemometer" type="custom" gz:type="anemometer">
///   <always_on>1</always_on>
///   <update_rate>30</update_rate>
///   <topic>anemometer</topic>
///   <gz:anemometer>
///     <noise type="gaussian">
///       <mean>0.2</mean>
///       <stddev>0.1</stddev>
///     </noise>
///   </gz:anemometer>
/// </sensor>
/// \endcode
///
/// # Published Topics
///
/// 1. ~/anemometer (msgs::Vector3d)
///   - header.stamp
///     The simulation time of the observation.
///   - header.data["frame_id"]
///     The name of the sensor.
///   - x, y, z
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
      public ISystemPreUpdate,
      public ISystemPostUpdate
{
  /// \brief Destructor.
  public: ~Anemometer() override;

  /// \brief Constructor.
  public: Anemometer();

  // Documentation inherited
  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void PostUpdate(
      const UpdateInfo &_info,
      const EntityComponentManager &_ecm) final;

  /// \brief Private data pointer.
  private: std::unique_ptr<AnemometerPrivate> dataPtr;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_ANEMOMETER_HH_
