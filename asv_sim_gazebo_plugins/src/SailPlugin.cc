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

#include "asv_sim_gazebo_plugins/SailPlugin.hh"
#include "asv_sim_gazebo_plugins/LiftDragModel.hh"
#include "asv_sim_gazebo_plugins/MessageTypes.hh"

#include <algorithm>
#include <functional>
#include <string>

#include <ignition/math/Pose3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/transport/transport.hh"

#include <boost/algorithm/string.hpp>

using namespace asv;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SailPlugin)

///////////////////////////////////////////////////////////////////////////////
// SailPluginPrivate

namespace asv
{
  /// \brief Type definition for a pointer to a LiftDrag message.
  typedef const boost::shared_ptr<
    const asv_msgs::msgs::LiftDrag>
      LiftDragPtr;


  class SailPluginPrivate
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
// SailPlugin

SailPlugin::~SailPlugin()
{
  // Reset connections and transport.
  this->data->updateConnection.reset();
  this->data->liftDragPub.reset();
  this->data->node->Fini();
}

SailPlugin::SailPlugin()
  : gazebo::ModelPlugin(),
  data(new SailPluginPrivate())
{
}

void SailPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "Invalid parameter _sdf");
  GZ_ASSERT(_model, "Invalid parameter _model");
  GZ_ASSERT(_model->GetWorld(), "Model has invalid world");

  // Capture model and sdf.
  this->data->sdf = _sdf;
  this->data->model = _model;
  this->data->world = _model->GetWorld();

  // Parameters
  this->LoadParam(_sdf, "cp", this->data->cp, this->data->cp);
 
  // Link
  std::string linkName;
  this->LoadParam(_sdf, "link_name", linkName, "");

  if (!linkName.empty())
  {
    this->data->link = this->data->model->GetLink(linkName);
    if (this->data->link == nullptr)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The SailPlugin will not generate forces\n";
    }
    else
    {
      this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SailPlugin::OnUpdate, this));
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

void SailPlugin::Reset()
{
  // Reset Time
  this->data->prevTime = this->data->world->SimTime();
}

std::string SailPlugin::GetTopic() const
{
  std::string topic;
  this->LoadParam(this->data->sdf, "topic", topic, "lift_drag");

  std::string topicName = "~";
  if (this->data->link != nullptr)
  {
    topicName += '/' + this->data->link->GetName();
  }
  topicName += '/' + topic;
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

void SailPlugin::OnUpdate()
{
  if (this->data->link == nullptr)
  {
    return;
  }
  // Wind velocity at the link origin (world frame).
  // We use this to approximate the wind at the centre of pressure
  auto& wind = this->data->world->Wind();
  auto velWind = wind.WorldLinearVel(this->data->link.get());

  // Linear velocity at the centre of pressure (world frame).
  auto velCp = this->data->link->WorldLinearVel(this->data->cp);

  // Free stream velocity at centre of pressure (world frame).
  auto vel = velWind - velCp;

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

  // @DEBUG_INFO
#if 0
  gzmsg << "Link:         " << this->data->link->GetName() << std::endl;
  // Scalars
  gzmsg << "u:            " << u                  << std::endl;
  gzmsg << "alpha:        " << alpha              << std::endl;
  // gzmsg << "cosAlpha:     " << cosAlpha           << std::endl;
  gzmsg << "cl:           " << cl                 << std::endl;
  gzmsg << "cd:           " << cd                 << std::endl;
  gzmsg << "||velWind||:  " << velWind.Length()   << std::endl;
  gzmsg << "||velCp||:    " << velCp.Length()     << std::endl;
  gzmsg << "||vel||:      " << vel.Length()       << std::endl;
  gzmsg << "||lift||:     " << lift.Length()      << std::endl;
  gzmsg << "||drag||:     " << drag.Length()      << std::endl;
  gzmsg << "||xr||:       " << xr.Length()        << std::endl;
  gzmsg << "||force||:    " << force.Length()     << std::endl;
  // Vectors
  gzmsg << "velWind:      " << velWind            << std::endl;
  gzmsg << "velCp:        " << velCp              << std::endl;
  gzmsg << "vel:          " << vel                << std::endl;
  // gzmsg << "velUnit:      " << velUnit            << std::endl;
  // gzmsg << "forwardI:     " << forwardI           << std::endl;
  // gzmsg << "upwardI:      " << upwardI            << std::endl;
  // gzmsg << "spanI:        " << spanI              << std::endl;
  // gzmsg << "velLD:        " << velLD              << std::endl;
  // gzmsg << "dragUnit:     " << dragUnit           << std::endl;
  // gzmsg << "liftUnit:     " << liftUnit           << std::endl;
  gzmsg << "lift:         " << lift               << std::endl;
  gzmsg << "drag:         " << drag               << std::endl;
  gzmsg << "xr:           " << xr                 << std::endl;
  gzmsg << "force:        " << force              << std::endl;
  gzmsg << "torque:       " << torque             << std::endl;
  gzmsg << "link          " << linkPose.Pos()     << std::endl;
  gzmsg << "com:          " << com                << std::endl;
  gzmsg << "cp:           " << cpWorld            << std::endl;
  gzmsg << std::endl;
#endif

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
