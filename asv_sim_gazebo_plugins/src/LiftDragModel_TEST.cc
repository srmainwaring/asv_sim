// Copyright (C) 2021-2023 Rhys Mainwaring
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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "asv/sim/LiftDragModel.hh"

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
TEST(LiftDragModel, Quadrants)
{
    // create SDF data
    sdf::SDFPtr model(new sdf::SDF());
    sdf::init(model);
    ASSERT_TRUE(sdf::readString(get_sdf_string(), model));

    sdf::ElementPtr plugin
        = model->Root()->GetElement("model")->GetElement("plugin");

    // create from SDF
    std::unique_ptr<asv::LiftDragModel> ld_model(
        asv::LiftDragModel::Create(plugin));

    gz::math::Vector3d velU(-10.0, 0.0, 0.0);
    gz::math::Vector3d lift;
    gz::math::Vector3d drag;
    double alpha = 0.0;
    double u = 0.0;
    double cl = 0.0;
    double cd = 0.0;

    {  // Case AoA = 0
        gz::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_DOUBLE_EQ(alpha, 0.0);
        EXPECT_DOUBLE_EQ(u, 10.0);
        EXPECT_DOUBLE_EQ(cl, 0.0);
        EXPECT_DOUBLE_EQ(cd, 0.0);

        EXPECT_DOUBLE_EQ(lift.X(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Y(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Z(), 0.0);

        EXPECT_DOUBLE_EQ(drag.X(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Y(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Z(), 0.0);
    }

    {  // Case AoA = 45 deg
        gz::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI/4.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_DOUBLE_EQ(alpha, M_PI/4.0);
        EXPECT_DOUBLE_EQ(u, 10.0);
        EXPECT_GE(cl, 0.0);
        EXPECT_GE(cd, 0.0);

        EXPECT_DOUBLE_EQ(lift.X(), 0.0);
        EXPECT_GE(lift.Y(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Z(), 0.0);

        EXPECT_LE(drag.X(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Y(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Z(), 0.0);
    }

    {  // Case AoA = 135 deg
        gz::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, 3*M_PI/4.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_DOUBLE_EQ(alpha, 3*M_PI/4);
        EXPECT_DOUBLE_EQ(u, 10.0);
        EXPECT_LE(cl, 0.0);
        EXPECT_GE(cd, 0.0);

        EXPECT_DOUBLE_EQ(lift.X(), 0.0);
        EXPECT_LE(lift.Y(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Z(), 0.0);

        EXPECT_LE(drag.X(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Y(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Z(), 0.0);
    }

    {  // Case AoA = 180 deg
        gz::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_DOUBLE_EQ(alpha, M_PI);
        EXPECT_DOUBLE_EQ(u, 10.0);
        EXPECT_DOUBLE_EQ(cl, 0.0);
        EXPECT_DOUBLE_EQ(cd, 0.0);

        EXPECT_DOUBLE_EQ(lift.X(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Y(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Z(), 0.0);

        EXPECT_DOUBLE_EQ(drag.X(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Y(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Z(), 0.0);
    }

    {  // Case AoA = -45 deg
        gz::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, -M_PI/4.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_DOUBLE_EQ(alpha, M_PI/4.0);
        EXPECT_DOUBLE_EQ(u, 10.0);
        EXPECT_LE(cl, 0.0);
        EXPECT_GE(cd, 0.0);

        EXPECT_DOUBLE_EQ(lift.X(), 0.0);
        EXPECT_LE(lift.Y(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Z(), 0.0);

        EXPECT_LE(drag.X(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Y(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Z(), 0.0);
    }

    {  // Case AoA = -135 deg
        gz::math::Pose3d bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, -3*M_PI/4.0);
        ld_model->Compute(velU, bodyPose, lift, drag, alpha, u, cl, cd);
        EXPECT_DOUBLE_EQ(alpha, 3*M_PI/4);
        EXPECT_DOUBLE_EQ(u, 10.0);
        EXPECT_GE(cl, 0.0);
        EXPECT_GE(cd, 0.0);

        EXPECT_DOUBLE_EQ(lift.X(), 0.0);
        EXPECT_GE(lift.Y(), 0.0);
        EXPECT_DOUBLE_EQ(lift.Z(), 0.0);

        EXPECT_LE(drag.X(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Y(), 0.0);
        EXPECT_DOUBLE_EQ(drag.Z(), 0.0);
    }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
