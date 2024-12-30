// Copyright (C) 2020-2023 Rhys Mainwaring
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

#include "asv/sim/GazeboRosAnemometer.h"

#include <memory>
#include <string>

#include <gazebo/physics/physics.hh>

// using namespace gazebo;

namespace asv
{
GazeboRosAnemometer::~GazeboRosAnemometer()
{
    // Reset connections and transport.
    update_connection_.reset();
}

GazeboRosAnemometer::GazeboRosAnemometer()
{
}

void GazeboRosAnemometer::Load(gazebo::sensors::SensorPtr _sensor,
    sdf::ElementPtr _sdf)
{

    // Get the parent sensor type
    sensor_ = std::dynamic_pointer_cast<sensors::AnemometerSensor>(_sensor);
    if (!sensor_)
    {
        gzthrow("GazeboRosAnemometer requires a Anemometer Sensor"
                " as its parent");
        return;
    }

    // Get the world
    std::string worldName = sensor_->WorldName();
    world_ = physics::get_world(worldName);

    // Default parameters
    namespace_.clear();
    topic_ = "wind/apparent";
    frame_id_ = "/anemometer_link";

    // Load parameters
    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->
            GetValue()->GetAsString();

    if (_sdf->HasElement("frameId"))
        frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

    if (_sdf->HasElement("topicName"))
        topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();


    // Make sure the ROS node for Gazebo has been initialised
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
            " unable to load plugin."
            " Load the Gazebo system plugin"
            " 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
        return;
    }

    nh_ = std::make_unique<ros::NodeHandle>(namespace_);
    publisher_ = nh_->advertise<geometry_msgs::Vector3Stamped>(topic_, 10);

    Reset();

    // Connect to the sensor update event
    update_connection_ = sensor_->ConnectUpdated(
        std::bind(&GazeboRosAnemometer::OnUpdate, this));

    // Activate the sensor
    sensor_->SetActive(true);
}

void GazeboRosAnemometer::Reset()
{
}

void GazeboRosAnemometer::OnUpdate()
{
    common::Time sim_time = world_->SimTime();

    // Activate sensor if it is not active
    if (!sensor_->IsActive())
    {
        sensor_->SetActive(true);
    }

    // Populate the message
    apparent_wind_.header.stamp.sec  = sim_time.sec;
    apparent_wind_.header.stamp.nsec = sim_time.nsec;

    auto vel = sensor_->ApparentWindVelocity();
    apparent_wind_.vector.x = vel.X();
    apparent_wind_.vector.y = vel.Y();
    apparent_wind_.vector.z = vel.Z();

    // Publish the apparent wind
    publisher_.publish(apparent_wind_);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosAnemometer)

}  // namespace asv
