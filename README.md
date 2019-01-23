# ASV Simulator

This package contains plugins and models that support the
simulation of surface vessels in Gazebo.  

## Installation

You will need a working installation of ROS and Gazebo in order to use this package.

The package was built and tested with:

- Gazebo version 9.4.1
- ROS Melodic Morenia

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
```

Create a catkin workspace:

```bash
mkdir asv_ws
cd asv_ws
mkdir src
catkin init
```

Clone the ASV Wave Simulator repository:

```bash
cd src
git clone ssh://rhys@diskstation.local:/volume1/git/asv_simulator
```

Compile the packages:

```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

or with tests:

```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --catkin-args run_tests
```

## Usage

Launch a Gazebo session with `roslaunch`:

```bash
roslaunch asv_wave_gazebo ocean_world.launch verbose:=true
```

## License

ASV Simulator is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ASV Simulator is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.
