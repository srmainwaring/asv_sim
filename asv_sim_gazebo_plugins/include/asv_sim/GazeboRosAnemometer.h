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

#ifndef ASV_SIM_GAZEBOROSANEMOMETER_H_
#define ASV_SIM_GAZEBOROSANEMOMETER_H_

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>

#include "asv_sim/AnemometerSensor.hh"


namespace asv
{
class GAZEBO_VISIBLE GazeboRosAnemometer : public gazebo::SensorPlugin
{
 public:
  virtual ~GazeboRosAnemometer();
  GazeboRosAnemometer();
  void Load(gazebo::sensors::SensorPtr _sensor,
      sdf::ElementPtr _sdf) override;
  void Reset() override;

 private:
  void OnUpdate();

  gazebo::physics::WorldPtr world_;
  gazebo::sensors::AnemometerSensorPtr sensor_;

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher publisher_;

  geometry_msgs::Vector3Stamped apparent_wind_;

  std::string namespace_;
  std::string topic_;
  std::string frame_id_;

  gazebo::event::ConnectionPtr update_connection_;
};

}  // namespace asv

#endif  // ASV_SIM_GAZEBOROSANEMOMETER_H_
