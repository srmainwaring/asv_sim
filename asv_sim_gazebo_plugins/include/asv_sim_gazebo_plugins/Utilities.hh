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

#ifndef ASV_SIM_GAZEBO_PLUGINS_UTILITIES_HH_
#define ASV_SIM_GAZEBO_PLUGINS_UTILITIES_HH_

#include <memory>
#include <string>

// #include <gazebo/common/Console.hh>
// #include <gazebo/common/Time.hh>
// #include <gazebo/msgs/msgs.hh>

// #include <ignition/math/Pose3.hh>
// #include <ignition/math/Vector3.hh>

// #include <sdf/sdf.hh>

namespace asv
{

#if 0
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
  const sdf::ElementPtr& _sdf,
  const std::string& _name,
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
void MsgParamSetValue(gazebo::msgs::Param& _param, const T& _value)
{
  gzwarn << "Using default template for MsgParamSetValue" << std::endl;
}

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A bool value to store in the parameter message.
template <>
void MsgParamSetValue<bool>(gazebo::msgs::Param& _param, const bool& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value An int value to store in the parameter message.
template <>
void MsgParamSetValue<int>(gazebo::msgs::Param& _param, const int& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A size_t value to store in the parameter message.
template <>
void MsgParamSetValue<size_t>(gazebo::msgs::Param& _param,
    const size_t& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A double value to store in the parameter message.
template <>
void MsgParamSetValue<double>(gazebo::msgs::Param& _param,
    const double& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A std::string value to store in the parameter message.
template <>
void MsgParamSetValue<std::string>(gazebo::msgs::Param& _param,
    const std::string& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A Time value to store in the parameter message.
template <>
void MsgParamSetValue<gazebo::common::Time>(gazebo::msgs::Param& _param,
    const gazebo::common::Time& _value);

/// \brief Template function specialisation for setting a value in
/// parameter message.
/// \param[out] _param Parameter message to set.
/// \param[in] _value A ignition::math::Vector3d value to store in the
/// parameter message.
template <>
void MsgParamSetValue<ignition::math::Vector3d>(gazebo::msgs::Param& _param,
    const ignition::math::Vector3d& _value);

/// \brief Template function for setting a value in parameter message.
/// \param[out] _param Parameter vector message to set.
/// \param[in] _paramName Parameter name whose value will be set.
/// \param[in] _value A value to store in the parameter message.
template <typename T>
void SetMsgParam(gazebo::msgs::Param_V& _msg, const std::string &_paramName,
    const T& _value)
{
  // Custom compare for params
  auto compare = [=](auto& _param)
  {
    return _param.name() == _paramName;
  };

  auto it = std::find_if(std::begin(_msg.param()),
      std::end(_msg.param()), compare);

  // Not found
  if (it == std::end(_msg.param()))
  {
    gzwarn << "Parameter <" << _paramName << "> not found: "
      <<  "Cannot set to " << _value << std::endl;
    return;
  }

  // Found
  auto index = std::distance(std::begin(_msg.param()), it);
  auto param = _msg.mutable_param(index);
  MsgParamSetValue<T>(*param, _value);
}
#endif

/// \brief Placeholder for minimal library.
void foo();

}  // namespace asv

#endif  // ASV_SIM_GAZEBO_PLUGINS_UTILITIES_HH_
