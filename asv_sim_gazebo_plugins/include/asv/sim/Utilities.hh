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

#ifndef ASV_SIM_UTILITIES_HH_
#define ASV_SIM_UTILITIES_HH_

#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs.hh>

#include <sdf/sdf.hh>

namespace asv
{

// This code modified from gazebo/common/Plugins.hh
// https://bitbucket.org/osrf/gazebo/src
//
// The original code is a member function of the Plugin base class.
// This code is modified so it can be used as a stand alone function.
/*
* Copyright (C) 2012 Open Source Robotics Foundation
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
/// \brief Load parameter value from _sdf and store it to the given
///        reference, using the supplied default value if the element in
///        _sdf is not found. A message is written using gzmsg reporting
///        whether the default value was used or not. String specialization
///        to allow accepting const char* values for std::string parameters.
/// \param[in] _sdf The SDF element of the plugin.
/// \param[in] _name Name of a tag inside the SDF.
/// \param[out] _target The reference to store the param value to.
/// \param[in] _defaultValue The default value.  template <typename V>
/// \param[in] _msgPrefix A prefix for the message. Default value is "".
template <typename V>
void LoadParam(
  const std::shared_ptr<const sdf::Element> &_sdf,
  const std::string &_name,
  V& _target,
  V _defaultValue = V(),
  const std::string& _msgPrefix = "")
{
  auto result = _sdf->Get<V>(_name, _defaultValue);

  if (!result.second)
  {
    gzmsg /* << this->handleName.c_str() << " Plugin missing <" */
      << _msgPrefix << " missing <"
      << _name.c_str() << ">, defaults to "
      << result.first << std::endl;
  }
  else
  {
    gzmsg /* << this->handleName.c_str() << " Plugin <" */
      << _msgPrefix << "<"
      << _name.c_str() << "> set to "
      << result.first << std::endl;
  }
  _target = result.first;
}

/// \brief Template function for setting a value in parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A value to store in the parameter message.
template <typename T>
void MsgParamSetValue(gz::msgs::Param &_param,
    const std::string &_name, const T &_value)
{
  gzwarn << "Using default template for MsgParamSetValue" << std::endl;
}

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A bool value to store in the parameter message.
template <>
void MsgParamSetValue<bool>(gz::msgs::Param &_param,
    const std::string &_name, const bool &_value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value An int value to store in the parameter message.
template <>
void MsgParamSetValue<int>(gz::msgs::Param &_param,
    const std::string &_name, const int &_value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A size_t value to store in the parameter message.
template <>
void MsgParamSetValue<size_t>(gz::msgs::Param &_param,
    const std::string &_name, const size_t& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A double value to store in the parameter message.
template <>
void MsgParamSetValue<double>(gz::msgs::Param &_param,
    const std::string &_name, const double& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A std::string value to store in the parameter message.
template <>
void MsgParamSetValue<std::string>(gz::msgs::Param &_param,
    const std::string &_name, const std::string &_value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A Time value to store in the parameter message.
// template <>
// void MsgParamSetValue<gz::common::Time>(gz::msgs::Param &_param,
//     const std::string &_name, const gz::common::Time &_value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A gz::math::Vector3d value to store in the
/// parameter message.
template <>
void MsgParamSetValue<gz::math::Vector3d>(gz::msgs::Param &_param,
    const std::string &_name, const gz::math::Vector3d &_value);

/// \brief Template function for setting a value in parameter message.
/// \param[out] _param Parameter vector message to set.
/// \param[in] _paramName Parameter name whose value will be set.
/// \param[in] _value A value to store in the parameter message.
// template <typename T>
// void SetMsgParam(gz::msgs::Param_V &_msg,
//     const std::string &_paramName,
//     const T &_value)
// {
//   // Custom compare for params
//   auto compare = [=](auto& _param)
//   {
//     return _param.name() == _paramName;
//   };

//   auto it = std::find_if(std::begin(_msg.param()),
//       std::end(_msg.param()), compare);

//   // Not found
//   if (it == std::end(_msg.param()))
//   {
//     gzwarn << "Parameter <" << _paramName << "> not found: "
//            <<  "Cannot set to " << _value << std::endl;
//     return;
//   }

//   // Found
//   auto index = std::distance(std::begin(_msg.param()), it);
//   auto param = _msg.mutable_param(index);
//   MsgParamSetValue<T>(*param, _value);
// }

}  // namespace asv

#endif  // ASV_SIM_UTILITIES_HH_
