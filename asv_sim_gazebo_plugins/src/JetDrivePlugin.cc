// Copyright (C) 2021  Rhys Mainwaring
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

#include "asv_sim_gazebo_plugins/JetDrivePlugin.hh"
#include "asv_sim_gazebo_plugins/MessageTypes.hh"
#include "asv_sim_gazebo_plugins/PluginUtils.hh"

#include <algorithm>
#include <functional>
#include <string>
#include <sstream>

#include <ignition/math.hh>
#include <ignition/msgs.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>

#include <boost/algorithm/string.hpp>

using namespace asv;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(JetDrivePlugin)

///////////////////////////////////////////////////////////////////////////////
// JetDrivePluginPrivate

namespace asv
{

class JetDrivePluginPrivate
{
    /// \brief SDF for this plugin.
    public: sdf::ElementPtr sdf;

    /// \brief Pointer to model containing plugin.
    public: physics::ModelPtr model;

    /// \brief Pointer to world.
    public: physics::WorldPtr world;

    /// \brief Pointer to impeller link.
    public: physics::LinkPtr impellerLink;

    /// \brief Pointer to the impeller joint.
    public: physics::JointPtr impellerJoint;

    /// \brief Pointer to direction unit link.
    public: physics::LinkPtr directionUnitLink;

    /// \brief Pointer to thrust reverser joint.
    public: physics::JointPtr thrustReverserJoint;

    /// \brief Connection to World Update events.
    public: event::ConnectionPtr updateConnection;

    /// \brief Gazebo transport node.
    public: transport::NodePtr node;

    /// \brief publish visuals
    public: transport::PublisherPtr visualPub;

    public: msgs::Visual visualMsg;

    // ignition markers (for movable text)
    // ignition::transport::Node ignNode;
    // ignition::msgs::Marker markerMsg;

    /// \brief Previous update time for publisher throttle.  
    public: common::Time lastDebugPubTime;
    public: common::Time lastVisualPubTime;

    // Parameters
    public: double impellerRpmMax = 100.0;
    public: double thrustMin = 0.0;
    public: double thrustMax = 20.0;
    public: double reverseThrustScaling = 0.5;

    // Constants - initial directions vectors for thrust / reverse thrust
    public: ignition::math::Vector3d unitThrustFwdBody = ignition::math::Vector3d(1, 0, 0);
    public: ignition::math::Vector3d unitThrustRvsBody = ignition::math::Vector3d(0, 0, -1);

    // Variables
    public: ignition::math::Vector3d thrustBody = ignition::math::Vector3d(0, 0, 0);
};
}

///////////////////////////////////////////////////////////////////////////////
// JetDrivePlugin

JetDrivePlugin::~JetDrivePlugin()
{
    // Reset connections and transport.
    this->data->updateConnection.reset();
    this->data->node->Fini();
}

JetDrivePlugin::JetDrivePlugin()
    : gazebo::ModelPlugin(),
    data(new JetDrivePluginPrivate())
{
}

void JetDrivePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_sdf, "Invalid parameter _sdf");
    GZ_ASSERT(_model, "Invalid parameter _model");

    // Capture model and sdf.
    this->data->sdf = _sdf;
    this->data->model = _model;
    this->data->world = _model->GetWorld();

    // Parameters
    std::string impellerLinkName;
    std::string impellerJointName;
    std::string directionUnitLinkName;
    std::string thrustReverserJointName;

    gazebo::LoadParam(this, _sdf, "impellerRpmMax", this->data->impellerRpmMax, this->data->impellerRpmMax);
    gazebo::LoadParam(this, _sdf, "thrustMin", this->data->thrustMin, this->data->thrustMin);
    gazebo::LoadParam(this, _sdf, "thrustMax", this->data->thrustMax, this->data->thrustMax);
    gazebo::LoadParam(this, _sdf, "reverseThrustScaling", this->data->reverseThrustScaling, this->data->reverseThrustScaling);

    gazebo::LoadParam(this, _sdf, "impellerLinkName", impellerLinkName, "impeller_link");
    gazebo::LoadParam(this, _sdf, "impellerJointName", impellerJointName, "impeller_joint");
    gazebo::LoadParam(this, _sdf, "directionUnitLinkName", directionUnitLinkName, "direction_unit_link");
    gazebo::LoadParam(this, _sdf, "thrustReverserJointName", thrustReverserJointName, "thrust_reverser_joint");

    // transport
    this->data->node = transport::NodePtr(new transport::Node());
    this->data->node->Init(this->data->world->Name());

    // links and joints
    this->data->impellerLink = this->data->model->GetLink(impellerLinkName);
    if (this->data->impellerLink == nullptr)
    {
        gzerr << "Link with name[" << impellerLinkName << "] not found. "
            << "The JetDrivePlugin will not generate forces\n";
        return;
    }
    this->data->directionUnitLink = this->data->model->GetLink(directionUnitLinkName);
    if (this->data->directionUnitLink == nullptr)
    {
        gzerr << "Link with name[" << directionUnitLinkName << "] not found. "
            << "The JetDrivePlugin will not generate forces\n";
        return;
    }
    this->data->impellerJoint = this->data->model->GetJoint(impellerJointName);
    if (this->data->impellerJoint == nullptr)
    {
        gzerr << "Joint with name[" << impellerJointName << "] not found. "
            << "The JetDrivePlugin will not generate forces\n";
        return;
    }
    this->data->thrustReverserJoint = this->data->model->GetJoint(thrustReverserJointName);
    if (this->data->thrustReverserJoint == nullptr)
    {
        gzerr << "Joint with name[" << thrustReverserJointName << "] not found. "
            << "The JetDrivePlugin will not generate forces\n";
        return;
    }

    // connections
    this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&JetDrivePlugin::OnUpdate, this));


    // publishers
    this->data->visualPub = this->data->node->Advertise<msgs::Visual>("~/visual", 10);

    // time
    this->data->lastDebugPubTime = this->data->world->SimTime();
    this->data->lastVisualPubTime = this->data->world->SimTime();

    // visuals
    auto linkName = this->data->directionUnitLink->GetName();
    this->data->visualMsg.set_name(linkName + "_THRUST_VISUAL");
    this->data->visualMsg.set_parent_name(this->data->directionUnitLink->GetScopedName());

    msgs::Geometry *geomMsg = this->data->visualMsg.mutable_geometry();
    geomMsg->set_type(msgs::Geometry::CYLINDER);
    geomMsg->mutable_cylinder()->set_radius(0.010);
    geomMsg->mutable_cylinder()->set_length(.01);

    this->data->visualMsg.mutable_material()->mutable_script()->set_name(
        "Gazebo/BlueGlow");

    // Set the pose of the visual relative to its parent
    msgs::Set(this->data->visualMsg.mutable_pose(),
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

    // Don't cast shadows
    this->data->visualMsg.set_cast_shadows(false);

    // markers (text)
    // ignition::transport::Node node;

    // this->data->markerMsg.set_ns(this->data->directionUnitLink->GetScopedName());
    // this->data->markerMsg.set_id(0);
    // this->data->markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    // this->data->markerMsg.set_type(ignition::msgs::Marker::SPHERE);
    // this->data->markerMsg.set_visibility(ignition::msgs::Marker_Visibility_GUI);
    // auto *matMsg = this->data->markerMsg.mutable_material();
    // matMsg->mutable_script()->set_name("Gazebo/Green");
    // ignition::msgs::Set(data->markerMsg.mutable_scale(),
    //                     ignition::math::Vector3d(0.2, 0.2, 0.2));

}
 
void JetDrivePlugin::Reset()
{
    // Reset Time
    this->data->lastDebugPubTime = this->data->world->SimTime();
    this->data->lastVisualPubTime = this->data->world->SimTime();
}

void JetDrivePlugin::OnUpdate()
{
    if (this->data->impellerLink == nullptr
        || this->data->impellerJoint == nullptr
        || this->data->directionUnitLink == nullptr
        || this->data->thrustReverserJoint == nullptr)
    {
        return;
    }

    // thrust magnitude is determined from the angular velocity of the impeller
    // the impeller rpm = ang_vel * 60 / (2 pi) 
    const double impellerAngVel = this->data->impellerJoint->GetVelocity(0);
    const double impellerRpm = impellerAngVel * 30.0 / M_PI;
    const double thrust = this->data->thrustMax
        * ignition::math::clamp(std::fabs(impellerRpm / this->data->impellerRpmMax), 0.0, 1.0);

    // thrust reverse joint angle and limits
    const double thetaMin = this->data->thrustReverserJoint->LowerLimit();
    const double thetaMax = this->data->thrustReverserJoint->UpperLimit();
    const double theta = this->data->thrustReverserJoint->Position(); 

    // fraction of reverse thrust (alpha in [0, 1])
    const double alpha = std::fabs(theta / (thetaMax - thetaMin));

    // friction reduction factor that reduces thrust when reversed
    const double scale = this->data->reverseThrustScaling;

    // create unit thrust vectors in body frame
    auto unitThrustFwdBody = this->data->unitThrustFwdBody;
    auto unitThrustRvsBody = this->data->unitThrustRvsBody;

    // thrust is directed in the opposite direction to the link rotation
    auto rot = ignition::math::Quaterniond(0, -theta, 0);
    unitThrustRvsBody = rot.RotateVector(unitThrustRvsBody);

    // Compute resultant thrust vector in body frame (direction unit link frame)
    this->data->thrustBody = (unitThrustFwdBody * (1.0 - alpha) + unitThrustRvsBody * alpha * scale) * thrust;

    // Rotate into world frame
    auto dirUnitLinkCoMWorldPose = this->data->directionUnitLink->WorldCoGPose();
    auto thrustWorld = dirUnitLinkCoMWorldPose.Rot().RotateVector(this->data->thrustBody);

    // Ensure no overflow.
    thrustWorld.Correct();

    // Add force and torque to link (applied to CoM in world frame).
    this->data->directionUnitLink->AddForce(thrustWorld);

    // Publish message / debug info at 2Hz
    auto simTime = this->data->world->SimTime();
    const double debugUpdateRate = 1.0;
    const double debugUpdateInterval = 1.0/debugUpdateRate;
    if ((simTime - this->data->lastDebugPubTime).Double() > debugUpdateInterval)
    {
        this->data->lastDebugPubTime = simTime;
#if 1
        gzdbg << "\n"
            << "impellerJoint:        " << this->data->impellerJoint->GetName() << "\n"
            << "impellerAngVel:       " << impellerAngVel << "\n"
            << "impellerRpm:          " << impellerRpm << "\n"
            << "thrustMin:            " << this->data->thrustMin << "\n"
            << "thrustMax:            " << this->data->thrustMax << "\n"
            << "thetaMin:             " << thetaMin << "\n"
            << "thetaMax:             " << thetaMax << "\n"
            << "theta:                " << theta << "\n"
            << "alpha:                " << alpha << "\n"
            << "scale:                " << scale << "\n"
            << "unit_thrust_fwd_body: " << unitThrustFwdBody << "\n"
            << "unit_thrust_rvs_body: " << unitThrustRvsBody << "\n"
            << "thrust_body:          " << this->data->thrustBody << "\n"
            << "||thrust||:           " << thrustWorld.Length() << "\n"
            << "thrust:               " << thrustWorld << "\n"
            << "\n";
#endif
    }

    // Publish visuals at 1Hz
    const double visualUpdateRate = 10.0;
    const double visualUpdateInterval = 1.0/visualUpdateRate;
    if ((simTime - this->data->lastVisualPubTime).Double() > visualUpdateInterval)
    {
        this->data->lastVisualPubTime = simTime;
        UpdateVisuals();
        this->data->visualPub->Publish(this->data->visualMsg);
        // this->data->ignNode.Request("/marker", this->data->markerMsg);
    }
}

// create visualization message for thrust
//
// See for example:
// gazebo/examples/plugins/model_visuals
//
void JetDrivePlugin::UpdateVisuals()
{
    const double thrust = this->data->thrustBody.Length();
    double scaledThrust = thrust / this->data->thrustMax;
    scaledThrust *= 0.5;

    msgs::Geometry *geomMsg = this->data->visualMsg.mutable_geometry();
    double offset = 0.0;
    if (scaledThrust == 0)
    {
        geomMsg->mutable_cylinder()->set_length(0.01);
    }
    else
    {
        geomMsg->mutable_cylinder()->set_length(std::fabs(scaledThrust));
        offset = 0.5 * scaledThrust * -1.0;
    }

    // calculate visual pose relative to the direction unit link
    auto cyl2BodyRot = ignition::math::Quaterniond(0, 0.5 * M_PI, 0);
    auto body2ThrustRot = ignition::math::Quaterniond();
    body2ThrustRot.From2Axes(
        this->data->unitThrustFwdBody,
        this->data->thrustBody);
    auto cyl2ThrustRot = cyl2BodyRot * body2ThrustRot.Inverse();

    auto pose1 = ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 0),
        cyl2ThrustRot
    );

    auto pose2 = ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, offset),
        ignition::math::Quaterniond()
    );

    auto pose = pose1 * pose2;

    // Set pose relative to parent
    msgs::Set(this->data->visualMsg.mutable_pose(), pose);

    // marker visual to display the force text...
    // auto dirUnitLinkCoMWorldPose  = this->data->directionUnitLink->WorldCoGPose();
    //
    // std::stringstream ss;
    // ss << std::fixed << std::setprecision(2) << this->data->thrust << " N";
    //
    // // this->data->markerMsg.mutable_text()->clear();
    // // this->data->markerMsg.set_text(ss.str());
    // ignition::msgs::Set(this->data->markerMsg.mutable_pose(),
    //     ignition::math::Pose3d(
    //         dirUnitLinkCoMWorldPose.Pos().X(),
    //         dirUnitLinkCoMWorldPose.Pos().Y(),
    //         dirUnitLinkCoMWorldPose.Pos().Z(),
    //         dirUnitLinkCoMWorldPose.Rot().W(),
    //         dirUnitLinkCoMWorldPose.Rot().X(),
    //         dirUnitLinkCoMWorldPose.Rot().Y(),
    //         dirUnitLinkCoMWorldPose.Rot().Z()));

    // this->data->ignNode.Request("/marker", this->data->markerMsg);
}
