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

// LoadParam template functions adapted from gazebo/common/Plugin.hh
// for use when GAZEBO_MAJOR_VERSION < 11

#ifndef ASV_SIM_PLUGINUTILS_HH_
#define ASV_SIM_PLUGINUTILS_HH_

#include <string>

#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>


namespace gazebo
{
/// \brief Load parameter value from _sdf and store it to the given
///        reference, using the supplied default value if the element in
///        _sdf is not found. A message is written using gzmsg reporting
///       whether the default value was used or not.
/// \param[in] _sdf The SDF element of the plugin.
/// \param[in] _name Name of a tag inside the SDF.
/// \param[out] _target The reference to store the param value to.
/// \param[in] _defaultValue The default value.
template <typename Plugin, typename V> void LoadParam(
    Plugin *plugin,
    const sdf::ElementPtr &_sdf,
    const std::string &_name, V &_target,
    V _defaultValue = V())
{
    auto result = _sdf->Get<V>(_name, _defaultValue);

    if (!result.second)
    {
        gzmsg << plugin->GetHandle().c_str() << " Plugin missing <"
            << _name.c_str() << ">, defaults to "
            << result.first << std::endl;
    }
    else
    {
        gzmsg << plugin->GetHandle().c_str() << " Plugin <"
            << _name.c_str() << "> set to "
            << result.first << std::endl;
    }
    _target = result.first;
}

/// \brief Load parameter value from _sdf and store it to the given
///        reference, using the supplied default value if the element in
///        _sdf is not found. A message is written using gzmsg reporting
///        whether the default value was used or not. String specialization
///        to allow accepting const char* values for std::string parameters.
/// \param[in] _sdf The SDF element of the plugin.
/// \param[in] _name Name of a tag inside the SDF.
/// \param[out] _target The reference to store the param value to.
/// \param[in] _defaultValue The default value.
template <typename Plugin> void LoadParam(
    Plugin* plugin,
    sdf::ElementPtr &_sdf,
    const std::string &_name, std::string &_target,
    const char* _defaultValue)
{
    LoadParam<Plugin, std::string>(plugin, _sdf, _name, _target, _defaultValue);
}

}  // namespace gazebo

#endif  // ASV_SIM_PLUGINUTILS_HH_
