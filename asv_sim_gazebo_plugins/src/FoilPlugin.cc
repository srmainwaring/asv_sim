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


// Portions of this code are modified from the OSRF LiftDragPlugin
/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "asv_sim_gazebo_plugins/FoilPlugin.hh"
#include "asv_sim_gazebo_plugins/LiftDragModel.hh"
#include "asv_sim_gazebo_plugins/MessageTypes.hh"
#include "asv_sim_gazebo_plugins/PluginUtils.hh"

#include <algorithm>
#include <functional>
#include <string>

#include <ignition/math/Pose3.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>

#include <boost/algorithm/string.hpp>

using namespace asv;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FoilPlugin)

///////////////////////////////////////////////////////////////////////////////
// FoilPluginPrivate

namespace asv
{
  /// \brief Type definition for a pointer to a LiftDrag message.
  typedef const boost::shared_ptr<
    const asv_msgs::msgs::LiftDrag>
      LiftDragPtr;


  class FoilPluginPrivate
  {
    /// \brief SDF for this plugin;
    public: sdf::ElementPtr sdf;

    /// \brief Pointer to model containing plugin.
    public: physics::ModelPtr model;

    /// \brief Pointer to link currently targeted by mud joint.
    public: physics::LinkPtr link;

    /// \brief Pointer to world.
    public: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    public: event::ConnectionPtr updateConnection;

    /// \brief Gazebo transport node.
    public: transport::NodePtr node;

    /// \brief Publish to topic "~/lift_drag".
    public: transport::PublisherPtr liftDragPub;

    /// \brief Previous update time for publisher throttle.  
    public: common::Time prevTime;

    // Parameters

    /// \brief center of pressure in link local coordinates
    public: ignition::math::Vector3d cp = ignition::math::Vector3d(0, 0, 0);

    public: std::unique_ptr<LiftDragModel> liftDrag;
  };
}

///////////////////////////////////////////////////////////////////////////////
// FoilPlugin

FoilPlugin::~FoilPlugin()
{
  // Reset connections and transport.
  this->data->updateConnection.reset();
  this->data->liftDragPub.reset();
  this->data->node->Fini();
}

FoilPlugin::FoilPlugin()
  : gazebo::ModelPlugin(),
  data(new FoilPluginPrivate())
{
}

void FoilPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "Invalid parameter _sdf");
  GZ_ASSERT(_model, "Invalid parameter _model");
  GZ_ASSERT(_model->GetWorld(), "Model has invalid world");

  // Capture model and sdf.
  this->data->sdf = _sdf;
  this->data->model = _model;
  this->data->world = _model->GetWorld();

  // Parameters
  std::string linkName;
#if (GAZEBO_MAJOR_VERSION < 11)
  gazebo::LoadParam(this, _sdf, "cp", this->data->cp, this->data->cp);
  gazebo::LoadParam(this, _sdf, "link_name", linkName, "");
#else
  this->LoadParam(_sdf, "cp", this->data->cp, this->data->cp);
  this->LoadParam(_sdf, "link_name", linkName, "");
#endif

  if (!linkName.empty())
  {
    this->data->link = this->data->model->GetLink(linkName);
    if (this->data->link == nullptr)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The FoilPlugin will not generate forces\n";
    }
    else
    {
      this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FoilPlugin::OnUpdate, this));
    }
  }

  // Transport
  this->data->node = transport::NodePtr(new transport::Node());
  this->data->node->Init(this->data->model->GetName());

  // Publishers
  std::string topic = this->GetTopic();
  this->data->liftDragPub 
    = this->data->node->Advertise<asv_msgs::msgs::LiftDrag>(topic);

  // Time
  this->data->prevTime = this->data->world->SimTime();

  // Lift / Drag model
  this->data->liftDrag.reset(LiftDragModel::Create(_sdf));
}

void FoilPlugin::Reset()
{
  // Reset Time
  this->data->prevTime = this->data->world->SimTime();
}

std::string FoilPlugin::GetTopic() const
{
  std::string topic;
#if (GAZEBO_MAJOR_VERSION < 11)
  gazebo::LoadParam(this, this->data->sdf, "topic", topic, "lift_drag");
#else
  this->LoadParam(this->data->sdf, "topic", topic, "lift_drag");
#endif

  std::string topicName = "~";
  if (this->data->link != nullptr)
  {
    topicName += '/' + this->data->link->GetName();
  }
  topicName += '/' + topic;
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

void FoilPlugin::OnUpdate()
{
  if (this->data->link == nullptr)
  {
    return;
  }
  // Linear velocity at the centre of pressure (world frame).
  auto velCp = this->data->link->WorldLinearVel(this->data->cp);

  // Free stream velocity at centre of pressure (world frame).
  auto vel = - velCp;

  // Pose of link origin and link CoM (world frame).
  auto linkPose = this->data->link->WorldPose();
  auto comPose  = this->data->link->WorldCoGPose();

  // Compute lift and drag
  double alpha=0, u=0, cl=0, cd=0;
  ignition::math::Vector3d lift = ignition::math::Vector3d::Zero;
  ignition::math::Vector3d drag = ignition::math::Vector3d::Zero;
  this->data->liftDrag->Compute(vel, linkPose, lift, drag, alpha, u, cl, cd);

  // Resultant force arising from lift and drag (world frame).
  auto force = lift + drag;

  // CoM (world frame).
  auto com = comPose.Pos();

  // Rotate the centre of pressure (CP) into the world frame.
  auto cpWorld = linkPose.Rot().RotateVector(this->data->cp);

  // Vector from CoM to CP.
  auto xr  = linkPose.Pos() + cpWorld - com;

  // Compute torque (about CoM in world frame)
  auto torque = xr.Cross(force);

  // Ensure no overflow.
  force.Correct();
  torque.Correct();

  // Add force and torque to link (applied to CoM in world frame).
  this->data->link->AddForce(force);
  this->data->link->AddTorque(torque);

  // Publish message
  const double updateRate = 50.0;
  const double updateInterval = 1.0/updateRate;
  common::Time simTime = this->data->world->SimTime();
  if ((simTime - this->data->prevTime).Double() < updateInterval)
  {
    return;
  }
  this->data->prevTime = simTime;

  // Save the time of the measurement
  asv_msgs::msgs::LiftDrag liftDragMsg;
  msgs::Set(liftDragMsg.mutable_time(), simTime);

  msgs::Set(liftDragMsg.mutable_lift(), lift);
  msgs::Set(liftDragMsg.mutable_drag(), drag);

  liftDragMsg.set_alpha(alpha);
  liftDragMsg.set_speed_ld(u);
  liftDragMsg.set_lift_coeff(cl);
  liftDragMsg.set_drag_coeff(cd);

  msgs::Set(liftDragMsg.mutable_vel(), vel);
  // msgs::Set(liftDragMsg.mutable_vel_ld(), velLD);
  msgs::Set(liftDragMsg.mutable_xr(), xr);
  msgs::Set(liftDragMsg.mutable_force(), force);
  msgs::Set(liftDragMsg.mutable_torque(), torque);

  // Publish the message if needed
  if (this->data->liftDragPub)
    this->data->liftDragPub->Publish(liftDragMsg);
  
  // @DEBUG_INFO
  // gzmsg << liftDragMsg.DebugString() << std::endl;
}
