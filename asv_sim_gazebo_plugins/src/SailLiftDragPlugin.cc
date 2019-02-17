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

// Code modified from OSRF LiftDragPlugin

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

#include "asv_sim_gazebo_plugins/SailLiftDragPlugin.hh"

#include <algorithm>
#include <functional>
#include <string>

#include <ignition/math/Pose3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/transport/transport.hh"

#include "lift_drag.pb.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SailLiftDragPlugin)

///////////////////////////////////////////////////////////////////////////////
// SailLiftDragPluginPrivate

namespace gazebo
{
  /// \brief Type definition for a pointer to a LiftDrag message.
  typedef const boost::shared_ptr<
    const asv_msgs::msgs::LiftDrag>
      LiftDragPtr;


  class SailLiftDragPluginPrivate
  {
    /// \brief SDF for this plugin;
    public: sdf::ElementPtr sdf;

    /// \brief Pointer to model containing plugin.
    public: physics::ModelPtr model;

    /// \brief Pointer to link currently targeted by mud joint.
    public: physics::LinkPtr link;

    /// \brief Pointer to world.
    public: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    public: physics::PhysicsEnginePtr physics;

    /// \brief Connection to World Update events.
    public: event::ConnectionPtr updateConnection;

    /// \brief Gazebo transport node.
    public: transport::NodePtr node;

    /// \brief Publish to topic "~/lift_drag".
    public: transport::PublisherPtr liftDragPub;

    /// \brief Previous update time for publisher throttle.  
    public: common::Time prevTime;

    // Parameters

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    public: double rho;

    /// \brief if the shape is aerodynamically radially symmetric about
    /// the forward direction. Defaults to false for wing shapes.
    /// If set to true, the upward direction is determined by the
    /// angle of attack.
    public: bool radialSymmetry = true;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight.
    public: ignition::math::Vector3d forward = ignition::math::Vector3d(1, 0, 0);

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    public: ignition::math::Vector3d upward = ignition::math::Vector3d(0, 0, 1);

    /// \brief center of pressure in link local coordinates
    public: ignition::math::Vector3d cp = ignition::math::Vector3d(0, 0, 0);

    /// \brief effective planeform surface area
    public: double area = 1.0;

    /// \brief Initial angle of attack
    public: double alpha0 = 0.0;

    /// \brief Coefficient of Lift / alpha slope.
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    public: double cla = 4.0/M_PI;

    /// \brief angle of attack when airfoil stalls
    public: double alphaStall = M_PI/4.0;

    /// \brief Cl-alpha rate after stall
    public: double claStall = -4.0/M_PI;


    /// \brief Coefficient of Drag / alpha slope.
    /// Drag = C_D * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    public: double cda = 0.0;
  };
}

///////////////////////////////////////////////////////////////////////////////
// SailLiftDragPlugin

SailLiftDragPlugin::~SailLiftDragPlugin()
{
  // Remove node from the topic manager.
  this->data->liftDragPub.reset();
  this->data->node->Fini();
}

SailLiftDragPlugin::SailLiftDragPlugin()
  : ModelPlugin(),
  data(new SailLiftDragPluginPrivate())
{
}

void SailLiftDragPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf,   "Invalid parameter _sdf");
  GZ_ASSERT(_model, "Invalid parameter _model");
  GZ_ASSERT(_model->GetWorld(), "Model has invalid world");

  // Capture model and sdf.
  this->data->sdf = _sdf;
  this->data->model = _model;
  this->data->world = _model->GetWorld();

  // Transport
  this->data->node = transport::NodePtr(new transport::Node());
  this->data->node->Init(this->data->world->Name());

  // Publishers
  this->data->liftDragPub 
    = this->data->node->Advertise<asv_msgs::msgs::LiftDrag>("~/lift_drag");

  // Time
  this->data->prevTime = this->data->world->SimTime();

  // Parameters
  this->LoadParam(_sdf, "fluid_density", this->data->rho, this->data->rho);

  this->LoadParam(_sdf, "radial_symmetry", this->data->radialSymmetry, this->data->radialSymmetry);
  this->LoadParam(_sdf, "forward", this->data->forward, this->data->forward);
  this->LoadParam(_sdf, "upward", this->data->upward, this->data->upward);
  this->LoadParam(_sdf, "cp", this->data->cp, this->data->cp);
  this->LoadParam(_sdf, "area", this->data->area, this->data->area);

  this->LoadParam(_sdf, "a0", this->data->alpha0, this->data->alpha0);
  this->LoadParam(_sdf, "cla", this->data->cla, this->data->cla);
  this->LoadParam(_sdf, "alpha_stall", this->data->alphaStall, this->data->alphaStall);
  this->LoadParam(_sdf, "cla_stall", this->data->claStall, this->data->claStall);

  this->LoadParam(_sdf, "cda", this->data->cda, this->data->cda);

  // Normalise
  this->data->forward.Normalize();
  this->data->upward.Normalize();

  // Link
  std::string linkName;
  this->LoadParam(_sdf, "link_name", linkName, "");

  if (!linkName.empty())
  {
    this->data->link = this->data->model->GetLink(linkName);
    if (this->data->link == nullptr)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The SailLiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SailLiftDragPlugin::OnUpdate, this));
    }
  }

}

// Documentation Inherited.
void SailLiftDragPlugin::Reset()
{
  // Reset Time
  this->data->prevTime = this->data->world->SimTime();
}

double SailLiftDragPlugin::LiftCoefficient(double _alpha) const
{
  // @TODO
  // This is not quite right as we want CL to be symmetric about PI/2 
  // but it's not about alphaStall...

  double alpha0     = this->data->alpha0;
  double cla        = this->data->cla;
  double alphaStall = this->data->alphaStall;
  double claStall   = this->data->claStall;

  auto f1 = [=](auto _x)
  {
    return cla * (_x - alpha0);
  };
  auto f2 = [=](auto _x)
  {
    return claStall * (_x - alphaStall) + f1(alphaStall);
  };
  auto f3 = [=](auto _x)
  {
    return - f1(_x - M_PI/2.0) + f2(M_PI/2.0);
  };
  auto f4 = [=](auto _x)
  {
    return - f2(_x - M_PI/2.0) + f3(M_PI/2 + alphaStall) + f1(alphaStall);
  };

  double cl = 0.0;
  if (_alpha < alphaStall)
  {
    cl = f1(_alpha);
  }
  else if (_alpha < M_PI/2.0)
  {
    cl = f2(_alpha);
  }
  else if (_alpha < M_PI/2.0 + alphaStall)
  {
    cl = f3(_alpha);
  }
  else
  {
    cl = f4(_alpha);
  }

  return cl;
}

double SailLiftDragPlugin::DragCoefficient(double _alpha) const
{
  double cda = this->data->cda;

  auto f1 = [=](auto _x)
  {
    return cda * _x;
  };
  auto f2 = [=](auto _x)
  {
    return - f1(_x - M_PI/2.0) + f1(M_PI/2.0);
  };

  double cd = 0.0;
  if (_alpha < M_PI/2.0)
  {
    cd = f1(_alpha);
  }
  else
  {
    cd = f2(_alpha);
  }

  return cd;
}

void SailLiftDragPlugin::OnUpdate()
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

  // Unit free stream velocity.
  auto velUnit = vel;
  velUnit.Normalize();

  // Avoid normalisation issues.
  if (vel.Length() <= 0.01)
    return;

  // Pose of link origin and link CoM (world frame).
  auto linkPose = this->data->link->WorldPose();
  auto comPose  = this->data->link->WorldInertialPose();

  // Rotate forward and upward vectors into world frame.
  auto forwardI = linkPose.Rot().RotateVector(this->data->forward);

  ignition::math::Vector3d upwardI;
  if (this->data->radialSymmetry)
  {
    // Use free stream velocity to determine upward direction
    // which is the component of free stream velocity perpendicular
    // to forward direction.
    auto tmp = forwardI.Cross(velUnit);
    upwardI = tmp.Cross(forwardI).Normalize();
  }
  else
  {
    upwardI = linkPose.Rot().RotateVector(this->data->upward);
  }

  // SpanwiseI: a vector normal to lift-drag-plane (world frame)
  auto spanwiseI = forwardI.Cross(upwardI).Normalize();

  // Compute the angle of attack, alpha.
  // 
  // This is the angle between the free stream velocity
  // projected into the lift-drag plane and the forward vector
  //
  auto velLD = vel - vel.Dot(spanwiseI) * velUnit;

  // Get direction of drag
  auto dragUnit = velLD;
  dragUnit.Normalize();

  // Get direction of lift
  auto liftUnit = dragUnit.Cross(spanwiseI);
  liftUnit.Normalize();

  // Compute angle of attack.
  double cosAlpha = -forwardI.Dot(dragUnit);
  double alpha = acos(cosAlpha);

  // Compute dynamic pressure.
  double speedLD = velLD.Length();
  double q = 0.5 * this->data->rho * speedLD * speedLD;

  // Compute lift coefficient.
  double cl = this->LiftCoefficient(alpha);

  // Compute lift force.
  auto lift = cl * q * this->data->area * liftUnit;

  // Compute drag coefficient.
  double cd = this->DragCoefficient(alpha);

  // Compute drag force.
  auto drag = cd * q * this->data->area * dragUnit;

  // Resultant force arising from lift and drag (world frame).
  auto force = lift + drag;

  // Vector from CoM to the centre of pressure (world frame).
  auto com = comPose.Pos();
  auto xr  = linkPose.Pos() + this->data->cp - com;

  // Compute torque (about CoM in world frame)
  auto torque = xr.Cross(force);

  // Ensure no overflow.
  force.Correct();
  torque.Correct();


  // @DEBUG_INFO
#if 0
  gzmsg << "velWind:      " << velWind    << std::endl;
  gzmsg << "velCp:        " << velCp      << std::endl;
  gzmsg << "vel:          " << vel        << std::endl;
  gzmsg << "velUnit:      " << velUnit    << std::endl;
  gzmsg << "forwardI:     " << forwardI   << std::endl;
  gzmsg << "upwardI:      " << upwardI    << std::endl;
  gzmsg << "spanwiseI:    " << spanwiseI  << std::endl;
  gzmsg << "velLD:        " << velLD      << std::endl;
  gzmsg << "dragUnit:     " << dragUnit   << std::endl;
  gzmsg << "liftUnit:     " << liftUnit   << std::endl;
  gzmsg << "cosAlpha:     " << cosAlpha   << std::endl;
  gzmsg << "alpha:        " << alpha      << std::endl;
  gzmsg << "speedLD:      " << speedLD    << std::endl;
  gzmsg << "cl:           " << cl         << std::endl;
  gzmsg << "cd:           " << cd         << std::endl;
  gzmsg << "lift:         " << lift       << std::endl;
  gzmsg << "drag:         " << drag       << std::endl;
  gzmsg << "xr:           " << xr         << std::endl;
  gzmsg << "force:        " << force      << std::endl;
  gzmsg << "torque:       " << torque     << std::endl;
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
  liftDragMsg.set_speed_ld(speedLD);
  liftDragMsg.set_lift_coeff(cl);
  liftDragMsg.set_drag_coeff(cd);

  msgs::Set(liftDragMsg.mutable_vel(), vel);
  msgs::Set(liftDragMsg.mutable_vel_ld(), velLD);
  msgs::Set(liftDragMsg.mutable_xr(), xr);
  msgs::Set(liftDragMsg.mutable_force(), force);
  msgs::Set(liftDragMsg.mutable_torque(), torque);

  // Publish the message if needed
  if (this->data->liftDragPub)
    this->data->liftDragPub->Publish(liftDragMsg);
  
  // @DEBUG_INFO
  // gzmsg << liftDragMsg.DebugString() << std::endl;


}
