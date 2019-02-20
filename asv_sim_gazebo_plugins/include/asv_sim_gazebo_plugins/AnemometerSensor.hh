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

/// \file AnemometerSensor.hh
/// \brief This file defines a Gazebo sensor used to measure wind speed and direction.

#ifndef _ASV_SIM_GAZEBO_PLUGINS_ANEMOMETER_SENSOR_HH_
#define _ASV_SIM_GAZEBO_PLUGINS_ANEMOMETER_SENSOR_HH_

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/util/system.hh>

#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include <memory>
#include <string>

namespace gazebo
{  
  namespace sensors
  {
    /// \brief Register the sensor with the server.
    GZ_SENSORS_VISIBLE void RegisterAnemometerSensor();

    /// \internal
    /// \brief Class to hold private data for the AnemometerSensor.
    class AnemometerSensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief AnemometerSensor to measure wind speed and direction.
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
    /// 1. ~/anemometer (gazebo::msgs::Param_V)
    ///   - time (gazebo::msgs::Time)
    ///     The simulation time of the observation. 
    ///   - true_wind (gazebo::msgs::Vector3d)
    ///     The true wind at the link origin. 
    ///   - apparent_wind (gazebo::msgs::Vector3d)
    ///     The apparent wind at the link origin
    ///     (i.e. true wind adjusted for the link velocity). 
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
    class GZ_SENSORS_VISIBLE AnemometerSensor: public Sensor
    {
      /// \brief Destructor.
      public: virtual ~AnemometerSensor() override;

      /// \brief Constructor.
      public: AnemometerSensor();

      // Documentation inherited.
      public: void Load(const std::string& _worldName, sdf::ElementPtr _sdf) override;

      // Documentation inherited.
      public: void Load(const std::string& _worldName) override;

      // Documentation inherited.
      public: void Init() override;

      // Documentation inherited.
      public: void Fini() override;

      /// \brief Create a topic string for the sensor.
      public: virtual std::string GetTopic() const;

      // Documentation inherited.
      public: bool UpdateImpl(const bool _force) override;

      /// \brief Accessor for the current true wind velocity in m s^-1.
      /// \return Current true wind velocity.
      public: ignition::math::Vector3d TrueWindVelocity() const;

      /// \brief Accessor for the current apparent wind velocity in m s^-1.
      /// \return Current apparent wind velocity.
      public: ignition::math::Vector3d ApparentWindVelocity() const;

      /// \internal
      /// \brief Pointer to the class private data.
      private: std::unique_ptr<AnemometerSensorPrivate> dataPtr;
    };
    /// \}
  } // namespace sensors
} // namespace gazebo

#endif // _ASV_SIM_GAZEBO_PLUGINS_ANEMOMETER_SENSOR_HH_
