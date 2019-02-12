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

#include "asv_sim_gazebo_plugins/AnemometerSensorPlugin.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/msgs/any.pb.h>
#include <gazebo/msgs/empty.pb.h>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/msgs/param.pb.h>
#include <gazebo/msgs/param_v.pb.h>

#include <ignition/math/Vector3.hh>

#include <algorithm>
#include <iostream>
#include <string>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_MODEL_PLUGIN(AnemometerSensorPlugin)

///////////////////////////////////////////////////////////////////////////////
// AnemometerSensorPluginPrivate

  /// \internal
  /// \brief Private data for the AnemometerSensorPlugin
  class AnemometerSensorPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief Model pointer.
    public: physics::ModelPtr model;

    /// \brief Set the wavefield to be static.
    public: bool isStatic = false;

    /// \brief Update rate.
    public: double updateRate = 30.0;

    /// \brief Previous update time.
    public: common::Time prevTime = 0.0;

    /// \brief Connection to the World Update events.
    public: event::ConnectionPtr updateConnection;

    /// \brief Gazebo transport node.
    public: transport::NodePtr gzNode;

    /// \brief Publish to gztopic "~/anemometer".
    public: transport::PublisherPtr anemometerPub;
  };

///////////////////////////////////////////////////////////////////////////////
// AnemometerSensorPlugin

  AnemometerSensorPlugin::~AnemometerSensorPlugin()
  {
    // Remove node from the topic manager.
    this->data->anemometerPub.reset();
    this->data->gzNode->Fini();
  }

  AnemometerSensorPlugin::AnemometerSensorPlugin() : 
    ModelPlugin(), 
    data(new AnemometerSensorPluginPrivate())
  {
  }

  void AnemometerSensorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Capture the Model pointer
    this->data->model = _model;
    this->data->world = _model->GetWorld();
    GZ_ASSERT(this->data->world != nullptr, "Model has invalid World");

    // Transport
    this->data->gzNode = transport::NodePtr(new transport::Node());
    this->data->gzNode->Init(this->data->world->Name());

    // Publishers
    this->data->anemometerPub 
      = this->data->gzNode->Advertise<msgs::Response>("~/anemometer");

    // Subscribers

    // Bind the update callback to the world update event 
    this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AnemometerSensorPlugin::OnUpdate, this));

    // Parameters
    // this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);
    // this->data->updateRate = Utilities::SdfParamDouble(*_sdf, "update_rate", 30.0);
  }

  void AnemometerSensorPlugin::Init()
  {
  }

  void AnemometerSensorPlugin::Reset()
  {
  }

  void AnemometerSensorPlugin::OnUpdate()
  {
    if (!this->data->isStatic)
    {
      // Throttle update
      auto updatePeriod = 1.0/this->data->updateRate;
      auto currentTime = this->data->world->SimTime();
      if ((currentTime - this->data->prevTime).Double() < updatePeriod)
      {
        return;
      }
      this->data->prevTime = currentTime; 

      // Update sensor...
    }
  }

} // namespace gazebo
