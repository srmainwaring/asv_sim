// Copyright (C) 2022-2023  Rhys Mainwaring
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

#include "WindControl.hh"

#include <gz/msgs/vector3d.pb.h>

#include <atomic>
#include <mutex>
#include <string>

#include <gz/plugin/Register.hh>

#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>

#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
inline namespace ASV_SIM_VERSION_NAMESPACE
{
  /// \brief Private data class for WindControl
  class WindControlPrivate
  {
    /// \brief Publish the wind velocity
    public: void PublishWindVelocity();

    /// \brief Mutex for topic
    public: std::mutex topicMutex;

    /// \brief Wind topic
    public: QString topic;

    /// \brief Wind speed
    public: std::atomic<double> windSpeed{0.0};

    /// \brief Wind angle
    public: std::atomic<double> windDirection{0.0};

    /// \brief Update on change
    // public: bool hasChanged{false};

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Name of the world
    public: std::string worldName;

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publisher
    public: gz::transport::Node::Publisher pub;
  };
}

/////////////////////////////////////////////////
void WindControlPrivate::PublishWindVelocity()
{
  // publish
  // if (this->hasChanged)
  {  
    // Advertise the topic
    auto topic = this->topic.toStdString();
    this->pub = this->node.Advertise<msgs::Vector3d>(topic);

    static bool warned{false};
    if (!this->pub && !warned)
    {
      warned = true;
      gzerr << "[WindControl]: Unable to publish on topic[" << topic << "]\n";
      return;
    }

    /// \todo(srmainwaring) what to do with z-component?
    // Convert zero reference from north to east and deg to rad.
    double windDirENU = this->windDirection - 90.0;
    double windDirRad = windDirENU * GZ_PI / 180.0;
    double vx = std::cos(windDirRad) * this->windSpeed;
    double vy = std::sin(windDirRad) * this->windSpeed;

    // Create and publish message
    msgs::Vector3d msg;
    msg.set_x(vx);
    msg.set_y(vy);
    this->pub.Publish(msg);
  }
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
WindControl::WindControl()
  : GuiSystem(), dataPtr(new WindControlPrivate)
{
}

/////////////////////////////////////////////////
WindControl::~WindControl() = default;

/////////////////////////////////////////////////
void WindControl::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
  {
    this->title = "Wind Control";
  }

  // Parameters from SDF
  if (_pluginElem)
  {
    {
      auto elem = _pluginElem->FirstChildElement("topic");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        this->dataPtr->topic = QString::fromStdString(elem->GetText());
      }
    }
    {
      if (auto elem = _pluginElem->FirstChildElement("wind_speed"))
      {
        double value{0.0};
        elem->QueryDoubleText(&value);
        this->dataPtr->windSpeed = value;
      }
    }
    {
      if (auto elem = _pluginElem->FirstChildElement("wind_direction"))
      {
        double value{0.0};
        elem->QueryDoubleText(&value);
        this->dataPtr->windDirection = value;
      }
    }
  }

}

/////////////////////////////////////////////////
void WindControl::Update(const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    // Get the name of the world
    if (this->dataPtr->worldName.empty())
    {
      _ecm.Each<components::World, components::Name>(
        [&](const Entity &,
            const components::World *,
            const components::Name *_name) -> bool
        {
          // Assume there's only one world
          this->dataPtr->worldName = _name->Data();
          return false;
        });
    }

    // Set up default topic name if not set
    if (this->dataPtr->topic.isEmpty())
    {
      std::string topic = transport::TopicUtils::AsValidTopic("/world/" +
          this->dataPtr->worldName + "/wind");
      static bool warned{false};
      if (topic.empty() && !warned)
      {
        warned = true;
        gzerr << "Failed to create topic for wind velocity\n";
        return;
      }
      // Set topic changed for GUI (but not publishing).
      this->dataPtr->topic = QString::fromStdString(topic);
      this->TopicChanged();
    }
    this->dataPtr->initialized = true;
  }

  // Sync the wind
  Entity windEntity = _ecm.EntityByComponents(components::Wind());

  auto windVel =
      _ecm.Component<components::WorldLinearVelocity>(windEntity)->Data();

  double vx = windVel.X();
  double vy = windVel.Y();
  double windSpeed = std::sqrt(vx * vx + vy * vy);
  double windDirRad = std::atan2(vy, vx);
  double windDirDeg = windDirRad * 180 / GZ_PI + 90;
  if (windDirDeg > 180.0)
    windDirDeg -= 360.0;
  if (windDirDeg < -180.0)
     windDirDeg += 360.0;
  this->dataPtr->windSpeed = windSpeed;
  this->dataPtr->windDirection = windDirDeg;
  this->WindSpeedChanged();
  this->WindDirectionChanged();
  gzdbg << "wind:      " << windVel << "\n"
        << "speed:     " << windSpeed << "\n"
        << "direction: " << windDirDeg << "\n";
}

/////////////////////////////////////////////////
QString WindControl::Topic() const
{
  return this->dataPtr->topic;
}

/////////////////////////////////////////////////
void WindControl::SetTopic(const QString &_topic)
{
  this->dataPtr->topic = _topic;
  // this->dataPtr->hasChanged = true;
  this->TopicChanged();

  gzmsg << "Wind Topic: " << _topic.toStdString() << "\n";
}

/////////////////////////////////////////////////
double WindControl::WindSpeed() const
{
  return this->dataPtr->windSpeed;
}

/////////////////////////////////////////////////
void WindControl::SetWindSpeed(double _windSpeed)
{
  this->dataPtr->windSpeed = _windSpeed;
  // this->dataPtr->hasChanged = true;
  this->WindSpeedChanged();

  gzmsg << "Wind Speed: " << _windSpeed << " (m/s)\n";

  this->dataPtr->PublishWindVelocity();
}

/////////////////////////////////////////////////
double WindControl::WindDirection() const
{
  return this->dataPtr->windDirection;
}

/////////////////////////////////////////////////
void WindControl::SetWindDirection(double _windDirection)
{
  this->dataPtr->windDirection = _windDirection;
  // this->dataPtr->hasChanged = true;
  this->WindDirectionChanged();

  gzmsg << "Wind Direction: " << _windDirection << " (deg)\n";

  this->dataPtr->PublishWindVelocity();
}

}  // namespace sim
}  // namespace gz

/////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(gz::sim::WindControl,
              gz::gui::Plugin)
