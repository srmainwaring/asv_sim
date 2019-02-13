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

#include "asv_sim_gazebo_plugins/RegisterSensorsPlugin.hh"
#include "asv_sim_gazebo_plugins/AnemometerSensor.hh"

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_SYSTEM_PLUGIN(RegisterSensorsPlugin)

  RegisterSensorsPlugin::~RegisterSensorsPlugin()
  {
  }

  RegisterSensorsPlugin::RegisterSensorsPlugin() : 
    SystemPlugin()
  {
  }

  void RegisterSensorsPlugin::Load(int _argc, char** _argv)
  {
    // Register the sensor with the server.
    gazebo::sensors::RegisterAnemometerSensor();
  }

  void RegisterSensorsPlugin::Init()
  {
  }

} // namespace asv
