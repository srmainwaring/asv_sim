# ASV Simulator

[![Ubuntu Jammy CI](https://github.com/srmainwaring/asv_sim/actions/workflows/ubuntu-jammy-ci.yml/badge.svg)](https://github.com/srmainwaring/asv_sim/actions/workflows/ubuntu-jammy-ci.yml)
[![Cpplint](https://github.com/srmainwaring/asv_sim/actions/workflows/ccplint.yml/badge.svg)](https://github.com/srmainwaring/asv_sim/actions/workflows/ccplint.yml)
[![Cppcheck](https://github.com/srmainwaring/asv_sim/actions/workflows/ccpcheck.yml/badge.svg)](https://github.com/srmainwaring/asv_sim/actions/workflows/ccpcheck.yml)

This package contains plugins and models for the simulation of surface vessels in Gazebo.

![rs750_ardupilot_v3_upwind](https://user-images.githubusercontent.com/24916364/228044489-b434b1ae-c30f-4676-9415-1719ee75479b.gif)

## Dependencies

- A working installation of [Gazebo Garden](https://gazebosim.org/docs/garden) or later including development symbols.

### Ubuntu

- Ubuntu 22.04 (Jammy)
- Gazebo Sim, version 7.1.0 (Garden)

### macOS

- macOS 12.6.2 (Monterey)
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

The package includes a custom Anemometer sensor to measure wind speed
and direction.

### Usage

Add the SDF for the sensor to a `<link>` element of your model.

```xml
<sensor name="anemometer" type="custom" gz:type="anemometer">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <topic>anemometer</topic>
  <gz:anemometer>
    <noise type="gaussian">
      <mean>0.2</mean>
      <stddev>0.1</stddev>
    </noise>
  </gz:anemometer>
</sensor>
```

### Published Topics

1. `~/anemometer` (`gz::msgs::Vector3d`)

  - `header.stamp` (`gz::msgs::Time`) \
    The simulation time of the observation.

  - `x, y, z` (`double`) \
    The apparent wind velocity components at the sensor origin
    in the world frame (i.e. true wind adjusted for the link velocity).

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
gz sim -v4 -s -r anemometer.sdf
```

The system plugin is registered with the simulation in the world file using:

```xml
<plugin filename="asv_sim2-anemometer-system"
  name="gz::sim::systems::Anemometer">
</plugin>
```

You should see a world containing a single block at the origin. 
The figure below shows the block falling to demonstate the effect
of motion on apparent wind:

![anemometer-falling](https://user-images.githubusercontent.com/24916364/224192127-adc7bab0-ade3-42ca-ae5b-1ea66548f392.jpg)


When the block is at rest with axis aligned with the world frame,
the true and apparent wind should be the same.

```bash
% gz topic -e -t /anemometer -n 1
header {
  stamp {
    sec: 61
    nsec: 369000000
  }
  data {
    key: "frame_id"
    value: "anemometer::base_link::anemometer"
  }
  data {
    key: "seq"
    value: "61368"
  }
}
x: -5
y: 1.4381672343972489e-17
z: -0.00099989893750004975
```

When the block is in motion, for instance by setting the `z` pose to `100` and letting it fall, the apparent wind will be adjusted for the object's motion.

```bash
gz topic -e -t /anemometer -n 1
header {
  stamp {
    sec: 58
    nsec: 596000000
  }
  data {
    key: "frame_id"
    value: "anemometer::base_link::anemometer"
  }
  data {
    key: "seq"
    value: "58595"
  }
}
x: -5
y: 1.959765959893313e-16
z: 25.675000101061269
```

## License

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.
