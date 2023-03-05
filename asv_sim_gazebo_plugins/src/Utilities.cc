// Copyright (C) 2019-2023 Rhys Mainwaring
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

#include "asv_sim_gazebo_plugins/Utilities.hh"

namespace asv
{
#if 0
template <>
void MsgParamSetValue<bool>(gazebo::msgs::Param& _param, const bool& _value)
{
  _param.mutable_value()->set_bool_value(_value);
}

template <>
void MsgParamSetValue<int>(gazebo::msgs::Param& _param, const int& _value)
{
  _param.mutable_value()->set_int_value(_value);
}

template <>
void MsgParamSetValue<size_t>(gazebo::msgs::Param& _param,
    const size_t& _value)
{
  _param.mutable_value()->set_int_value(_value);
}

template <>
void MsgParamSetValue<double>(gazebo::msgs::Param& _param,
    const double& _value)
{
  _param.mutable_value()->set_double_value(_value);
}

template <>
void MsgParamSetValue<std::string>(gazebo::msgs::Param& _param,
    const std::string& _value)
{
  _param.mutable_value()->set_string_value(_value);
}

template <>
void MsgParamSetValue<gazebo::common::Time>(gazebo::msgs::Param& _param,
    const gazebo::common::Time& _value)
{
  _param.mutable_value()->mutable_time_value()->set_sec(_value.sec);
  _param.mutable_value()->mutable_time_value()->set_nsec(_value.nsec);
}

template <>
void MsgParamSetValue<ignition::math::Vector3d>(gazebo::msgs::Param& _param,
  const ignition::math::Vector3d& _value)
{
  _param.mutable_value()->mutable_vector3d_value()->set_x(_value.X());
  _param.mutable_value()->mutable_vector3d_value()->set_y(_value.Y());
  _param.mutable_value()->mutable_vector3d_value()->set_z(_value.Z());
}
#endif

/////////////////////////////////////////////////
void foo()
{
}

}  // namespace asv
