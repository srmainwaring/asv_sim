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

#include "asv_sim_gazebo_plugins/LiftDragModel.hh"

#include <sdf/Model.hh>

#include <gtest/gtest.h>
#include <memory>

using namespace asv;

///////////////////////////////////////////////////////////////////////////////
// Define tests

std::string get_sdf_string()
{
  std::ostringstream stream;
  stream
    << "<sdf version='1.6'>"
    << "<model name='wing_sail'>"
    << "    <plugin name='wing_sail_liftdrag' filename='libSailPlugin.so'>"
    << "        <a0>0.0</a0>"
    << "        <cla>6.2832</cla>"
    << "        <alpha_stall>0.1592</alpha_stall>"
    << "        <cla_stall>-0.7083</cla_stall>"
    << "        <cda>0.63662</cda>"
    << "        <area>0.4858</area>"
    << "        <fluid_density>1.2</fluid_density>"
    << "        <forward>1 0 0</forward>"
    << "        <upward>0 1 0</upward>"
    << "        <cp>-0.05 0.0 0.65</cp>"
    << "        <link_name>wing_sail_link</link_name>"
    << "        <radial_symmetry>true</radial_symmetry>"
    << "        <topic>lift_drag</topic>"
    << "    </plugin>"
    << "</model>"
    << "</sdf>";

  return stream.str();
}

TEST(LiftDragModel, LiftCoefficients)
{
    // create SDF data
    sdf::SDFPtr model(new sdf::SDF());
    sdf::init(model);
    ASSERT_TRUE(sdf::readString(get_sdf_string(), model));

    sdf::ElementPtr plugin
        = model->Root()->GetElement("model")->GetElement("plugin");
    
    // create from SDF
    std::unique_ptr<LiftDragModel> ld_model(LiftDragModel::Create(plugin));     

    ignition::math::Vector3d lift;
    ignition::math::Vector3d drag;
    double alpha=0.0;
    double u=0.0;
    double cl=0.0;
    double cd=0.0;

    { // Case alpha = 0
        ignition::math::Vector3d velU(-10.0, 0.0, 0.0);
        ignition::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_EQ(alpha, 0.0);
        EXPECT_EQ(u, 10.0);
        EXPECT_EQ(cl, 0.0);
        EXPECT_EQ(cd, 0.0);
        EXPECT_EQ(lift.X(), 0.0);
        EXPECT_EQ(lift.Y(), 0.0);
        EXPECT_EQ(lift.Z(), 0.0);
        EXPECT_EQ(drag.X(), 0.0);
        EXPECT_EQ(drag.Y(), 0.0);
        EXPECT_EQ(drag.Z(), 0.0);
    }

    { // Case alpha = 45 deg
        ignition::math::Vector3d velU(-10.0, 0.0, 0.0);
        ignition::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI/4.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_GE(alpha, 0.0);
        EXPECT_EQ(u, 10.0);
        EXPECT_GE(cl, 0.0);
        EXPECT_GE(cd, 0.0);
        EXPECT_EQ(lift.X(), 0.0);
        EXPECT_GE(lift.Y(), 0.0);
        EXPECT_EQ(lift.Z(), 0.0);
        EXPECT_LE(drag.X(), 0.0);
        EXPECT_EQ(drag.Y(), 0.0);
        EXPECT_EQ(drag.Z(), 0.0);
    }

    { // Case alpha = 135 deg
        ignition::math::Vector3d velU(-10.0, 0.0, 0.0);
        ignition::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, 3*M_PI/4.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_GE(alpha, 0.0);
        EXPECT_EQ(u, 10.0);
        EXPECT_GE(cl, 0.0);
        EXPECT_GE(cd, 0.0);
        EXPECT_EQ(lift.X(), 0.0);
        EXPECT_GE(lift.Y(), 0.0);
        EXPECT_EQ(lift.Z(), 0.0);
        EXPECT_LE(drag.X(), 0.0);
        EXPECT_EQ(drag.Y(), 0.0);
        EXPECT_EQ(drag.Z(), 0.0);
    }

}

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

