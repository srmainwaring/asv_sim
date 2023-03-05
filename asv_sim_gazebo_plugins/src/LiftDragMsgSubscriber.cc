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

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

#include <boost/program_options.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

#include "asv_sim_gazebo_plugins/MessageTypes.hh"

// using namespace gazebo;
namespace po = boost::program_options;

///////////////////////////////////////////////////////////////////////////////
/// Ignition Transport Tutorial
/// Signal handler example
/// https://ignitionrobotics.org/api/transport/6.0/messages.html
///
static std::atomic<bool> g_terminatePub(false);

void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Callback for topic "~/lift_drag".
///

void OnLiftDragMsg(asv::LiftDragPtr &_msg)
{
  std::cout <<  "---\n";
  std::cout <<  _msg->DebugString();
}

///////////////////////////////////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  try
  {
    // Copyright notice.
    std::cout
      << "ASV Simulator: lift drag subscriber.\n"
      << "Copyright (C) 2019  Rhys Mainwaring.\n"
      << "Released under the GNU General Public License.\n\n";

    // Program options
    po::options_description options(
        "Subscribe to lift drag messages (asv_msgs/LiftDrag)");

    options.add_options()
      ("help,h",
        "Dispay this help screen.")
      ("topic,t", po::value<std::string>(),
        "The topic to subscribe to.");

    po::variables_map vm;
    po::store(po::parse_command_line(_argc, _argv, options), vm);
    po::notify(vm);

    if (vm.count("help") || vm.empty())
    {
      std::cout << options << std::endl;
      return 0;
    }

    // get the topic
    if (!vm.count("topic"))
    {
        std::cout << "No topic provided" << std::endl;
        return 0;
    }
    auto topic = vm["topic"].as<std::string>();

    // Signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Transport
    transport::init();
    transport::run();
    transport::NodePtr node(new transport::Node());
    node->Init();

    transport::SubscriberPtr liftDragSub =
      node->Subscribe(topic, &OnLiftDragMsg);

    // Listen until interrupt
    while (!g_terminatePub)
    {
      // Listen
    }

    // Tear down
    liftDragSub.reset();
    transport::fini();
  }
  catch(const gazebo::common::Exception &_e)
  {
    std::cout << _e.GetErrorStr() << std::endl;
    transport::fini();
    return -1;
  }
  catch(const std::exception &_e)
  {
    std::cout << _e.what() << std::endl;
    transport::fini();
    return -1;
  }
  catch(...)
  {
    std::cout << "Unknown Error" << std::endl;
    transport::fini();
    return -1;
  }

  return 0;
}
