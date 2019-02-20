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

#ifndef _ASV_SIM_GAZEBO_PLUGINS_UTILITIES_HH_
#define _ASV_SIM_GAZEBO_PLUGINS_UTILITIES_HH_

// #include <gazebo/util/system.hh>
// #include <gazebo/gazebo_config.hh>
#include <gazebo/common/Console.hh>
// #include <gazebo/common/Exception.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include <memory>

namespace asv
{
  // Code from gazebo/common/Plugins.hh
  template <typename V>
  void LoadParam(
    const sdf::ElementPtr& _sdf,
    const std::string& _name,
    V& _target,
    V _defaultValue = V())
  {
    auto result = _sdf->Get<V>(_name, _defaultValue);

    if (!result.second)
    {
      gzmsg << /*this->handleName.c_str() << " Plugin missing <" */ "<"
        << _name.c_str() << ">, defaults to "
        << result.first << std::endl;
    }
    else
    {
      gzmsg << /*this->handleName.c_str() << " Plugin <" */ "<"
        << _name.c_str() << "> set to "
        << result.first << std::endl;
    }
    _target = result.first;    
  }

} // namespace asv

#endif // _ASV_SIM_GAZEBO_PLUGINS_UTILITIES_HH_
