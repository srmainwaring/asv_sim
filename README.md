# ASV Simulator

[![Ubuntu Jammy CI](https://github.com/srmainwaring/asv_sim/actions/workflows/ubuntu-jammy-ci.yml/badge.svg)](https://github.com/srmainwaring/asv_sim/actions/workflows/ubuntu-jammy-ci.yml)
[![Cpplint](https://github.com/srmainwaring/asv_sim/actions/workflows/ccplint.yml/badge.svg)](https://github.com/srmainwaring/asv_sim/actions/workflows/ccplint.yml)
[![Cppcheck](https://github.com/srmainwaring/asv_sim/actions/workflows/ccpcheck.yml/badge.svg)](https://github.com/srmainwaring/asv_sim/actions/workflows/ccpcheck.yml)

This package contains plugins and models for the simulation of surface vessels in Gazebo.

## Dependencies

- A working installation of [Gazebo Garden](https://gazebosim.org/docs/garden) or later including development symbols.

### Ubuntu

- Ubuntu 22.04 (Jammy)
- Gazebo Sim, version 7.1.0 (Garden)

### macOS

- macOS 12.6 (Monterey)
- Gazebo Sim, version 7.1.0 (Garden)

## Installation

### Create a workspace

```bash
mkdir -p gz_ws/src
```

### Clone and build the package

Clone the `asv_sim` repository:

```bash
cd ~/gz_ws/src
git clone https://github.com/srmainwaring/asv_sim.git
```

Compile the package:

#### Ubuntu

```bash
colcon build --symlink-install --merge-install --cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DBUILD_TESTING=ON \
-DCMAKE_CXX_STANDARD=17
```

Source the workspace:

```bash
source ./install/setup.bash
```

#### macOS

```bash
colcon build --symlink-install --merge-install --cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DBUILD_TESTING=ON \
-DCMAKE_CXX_STANDARD=17 \
-DCMAKE_MACOSX_RPATH=FALSE \
-DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib
```

Source the workspace:

```bash
source ./install/setup.zsh
```

## Usage

### Set environment variables

```bash
# for future use - to support multiple Gazebo versions
export GZ_VERSION=garden

# not usually required as should default to localhost address
export GZ_IP=127.0.0.1

# ensure the model and world files are found
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/gz_ws/src/asv_sim/asv_sim_gazebo/worlds

# ensure the system plugins are found
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/gz_ws/src/asv_sim/install/lib
```

## Anemometer Sensor

AnemometerSensor to measure wind speed and direction.

### Usage

Add the SDF for the sensor to a `<link>` element of your model.

```xml
<sensor name="anemometer_sensor" type="anemometer">
  <always_on>true</always_on>
  <update_rate>50</update_rate>
  <topic>anemometer</topic>
</sensor>
```

### Published Topics

1. `~/anemometer` (`gazebo::msgs::Param_V`)

  - `time` (`gazebo::msgs::Time`) \
    The simulation time of the observation.

  - `true_wind` (`gazebo::msgs::Vector3d`) \
    The true wind at the link origin.

  - `apparent_wind` (`gazebo::msgs::Vector3d`) \
    The apparent wind at the link origin
    (i.e. true wind adjusted for the link velocity).

### Parameters

1. `<always_on>` (`bool`, default: `false`) \
  Standard `<sensor>` parameter.
  See [SDF documentation](http://sdformat.org/spec?ver=1.6&elem=sensor) for details.

2. `<update_rate>` (`double`, default: `0`) \
  Standard `<sensor>` parameter.
  See [SDF documentation](http://sdformat.org/spec?ver=1.6&elem=sensor) for details.

3. `<topic>` (`string`, default: `~/anemometer`) \
  Standard `<sensor>` parameter.
  See [SDF documentation](http://sdformat.org/spec?ver=1.6&elem=sensor) for details.

## Anemometer Example

To run the example:

```bash
roslaunch asv_sim_gazebo anemometer_demo_world.launch verbose:=true
```

The launch file loads the `RegisterSensorsPlugin` system plugin using
the parameter:  

```xml
  <arg name="extra_gazebo_args" default="--server-plugin libRegisterSensorsPlugin.so" />
```

You should see a world containing a single block at the origin. 
The figure below shows the block falling to demonstate the effect
of motion on apparent wind:

![anemometer](https://user-images.githubusercontent.com/24916364/224131738-0277d78e-8ab1-4c07-bf14-072b1e3fed19.jpg)

Open the Topic Visualization window and select the `anemometer` topic:

![Anemometer Topic](https://github.com/srmainwaring/asv_sim/wiki/images/anemometer_topic.jpg)

When the block is at rest with axis aligned with the world frame,
the true and apparent wind should be the same. When the block is in motion,
for instance by setting the `z` pose to `100` and letting it fall, the
apparent wind will be adjusted for the object's motion.

![Anemometer Rest](https://github.com/srmainwaring/asv_sim/wiki/images/anemometer_topic_view_rest.jpg)
![Anemometer Falling](https://github.com/srmainwaring/asv_sim/wiki/images/anemometer_topic_view_falling.jpg)


## License

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.
