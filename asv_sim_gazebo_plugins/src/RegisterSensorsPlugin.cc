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
#include "asv_sim_gazebo_plugins/MessageTypes.hh"

#include <gazebo/msgs/MsgFactory.hh>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_SYSTEM_PLUGIN(RegisterSensorsPlugin)

////////////////////////////////////////////////////////////////////////////////
// Register Messages

  // GZ_REGISTER_STATIC_MSG("asv_msgs.msgs.Anemometer", Anemometer)
  GAZEBO_VISIBLE boost::shared_ptr<google::protobuf::Message> NewAnemometer()
  {
    return boost::shared_ptr<asv_msgs::msgs::Anemometer>(
      new asv_msgs::msgs::Anemometer());
  } 
  class GAZEBO_VISIBLE MsgAnemometer
  {
    public: MsgAnemometer()
    {
      gazebo::msgs::MsgFactory::RegisterMsg("asv_msgs.msgs.Anemometer", NewAnemometer);
      gzmsg << "RegisterMsg: Type: " << "asv_msgs.msgs.Anemometer" << std::endl;
    }
  };
  static MsgAnemometer GzAnemometerMsgInitializer;

  // GZ_REGISTER_STATIC_MSG("asv_msgs.msgs.LiftDrag", LiftDrag)
  GAZEBO_VISIBLE boost::shared_ptr<google::protobuf::Message> NewLiftDrag()
  {
    return boost::shared_ptr<asv_msgs::msgs::LiftDrag>(
      new asv_msgs::msgs::LiftDrag());
  } 
  class GAZEBO_VISIBLE MsgLiftDrag
  {
    public: MsgLiftDrag()
    {
      gazebo::msgs::MsgFactory::RegisterMsg("asv_msgs.msgs.LiftDrag", NewLiftDrag);
      gzmsg << "RegisterMsg: Type: " << "asv_msgs.msgs.LiftDrag" << std::endl;
    }
  };
  static MsgLiftDrag GzLiftDragMsgInitializer;

////////////////////////////////////////////////////////////////////////////////
// RegisterSensorsPlugin

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
    gzmsg << "RegisterSensor: Type: " << "Anemometer" << std::endl;
  }

  void RegisterSensorsPlugin::Init()
  {
  }

} // namespace asv
