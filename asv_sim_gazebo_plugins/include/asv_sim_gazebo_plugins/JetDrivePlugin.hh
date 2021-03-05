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

#ifndef _ASV_SIM_GAZEBO_PLUGINS_JET_DRIVE_PLUGIN_HH_
#define _ASV_SIM_GAZEBO_PLUGINS_JET_DRIVE_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace asv
{
class JetDrivePluginPrivate;

/// \brief A plugin that simulates thrust for a jet drive
class GAZEBO_VISIBLE JetDrivePlugin : public gazebo::ModelPlugin
{
    /// \brief Destructor.
    public: virtual ~JetDrivePlugin();

    /// \brief Constructor.
    public: JetDrivePlugin();

    // Documentation Inherited.
    public: void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation Inherited.
    public: void Reset() override;

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief
    protected: void UpdateVisuals();


    /// \internal
    /// \brief Pointer to the class private data.
    private: std::unique_ptr<JetDrivePluginPrivate> data;
};

} // namespace asv

#endif // _ASV_SIM_GAZEBO_PLUGINS_JET_DRIVE_PLUGIN_HH_
