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

#ifndef ASV_SIM_GAZEBO_PLUGINS_SAILLIFTDRAG_HH_
#define ASV_SIM_GAZEBO_PLUGINS_SAILLIFTDRAG_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

// Forward declarations.
class SailLiftDragPrivate;

/// \brief A plugin that simulates lift and drag on a sail
/// in the presence of wind.
class SailLiftDrag
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  /// \brief Destructor.
  public: virtual ~SailLiftDrag();

  /// \brief Constructor.
  public: SailLiftDrag();

  // Documentation inherited
  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) final;

  /// Documentation inherited
  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  // // Documentation Inherited.
  // public: void Load(gazebo::physics::ModelPtr _model,
  //     sdf::ElementPtr _sdf) override;

  // // Documentation Inherited.
  // public: void Reset() override;

  // /// \brief Create a topic string for the sensor.
  // public: virtual std::string GetTopic() const;

  // /// \brief Callback for World Update events.
  // protected: virtual void OnUpdate();

  /// \brief Private data pointer.
  private: std::unique_ptr<SailLiftDragPrivate> dataPtr;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // ASV_SIM_GAZEBO_PLUGINS_SAILLIFTDRAG_HH_
