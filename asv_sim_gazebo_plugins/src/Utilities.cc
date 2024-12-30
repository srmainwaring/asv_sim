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

#include "asv/sim/Utilities.hh"

#include <string>

namespace asv
{
template <>
void MsgParamSetValue<bool>(gz::msgs::Param &_param,
    const std::string &_name, const bool &_value)
{
  gz::msgs::Any value;
  value.set_type(gz::msgs::Any::DOUBLE);
  value.set_bool_value(_value);
  (*_param.mutable_params())[_name] = value;
}

/////////////////////////////////////////////////
template <>
void MsgParamSetValue<int>(gz::msgs::Param &_param,
    const std::string &_name, const int &_value)
{
  gz::msgs::Any value;
  value.set_type(gz::msgs::Any::INT32);
  value.set_int_value(_value);
  (*_param.mutable_params())[_name] = value;
}

/////////////////////////////////////////////////
template <>
void MsgParamSetValue<size_t>(gz::msgs::Param &_param,
    const std::string &_name, const size_t &_value)
{
  /// \todo(srmainwaring) truncates
  gz::msgs::Any value;
  value.set_type(gz::msgs::Any::INT32);
  value.set_int_value(_value);
  (*_param.mutable_params())[_name] = value;
}

/////////////////////////////////////////////////
template <>
void MsgParamSetValue<double>(gz::msgs::Param &_param,
    const std::string &_name, const double &_value)
{
  gz::msgs::Any value;
  value.set_type(gz::msgs::Any::DOUBLE);
  value.set_double_value(_value);
  (*_param.mutable_params())[_name] = value;
}

/////////////////////////////////////////////////
template <>
void MsgParamSetValue<std::string>(gz::msgs::Param &_param,
    const std::string &_name, const std::string &_value)
{
  gz::msgs::Any value;
  value.set_type(gz::msgs::Any::STRING);
  value.set_string_value(_value);
  (*_param.mutable_params())[_name] = value;
}

/////////////////////////////////////////////////
// template <>
// void MsgParamSetValue<gz::common::Time>(gz::msgs::Param &_param,
//     const std::string &_name, const gz::common::Time &_value)
// {
//   _param.mutable_value()->mutable_time_value()->set_sec(_value.sec);
//   _param.mutable_value()->mutable_time_value()->set_nsec(_value.nsec);
// }

/////////////////////////////////////////////////
template <>
void MsgParamSetValue<gz::math::Vector3d>(gz::msgs::Param &_param,
  const std::string &_name, const gz::math::Vector3d &_value)
{
  gz::msgs::Any value;
  value.set_type(gz::msgs::Any::VECTOR3D);
  value.mutable_vector3d_value()->set_x(_value.X());
  value.mutable_vector3d_value()->set_y(_value.Y());
  value.mutable_vector3d_value()->set_z(_value.Z());
  (*_param.mutable_params())[_name] = value;
}

}  // namespace asv
