# ASV Simulator

This package contains plugins and models for the simulation of surface vessels in Gazebo.  

## Dependencies

You will need a working installation of ROS and Gazebo in order to use this package.
It has been tested with:

- Gazebo version 9.4.1
- ROS Melodic Morenia
- OSX 10.11.6

## Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
source /usr/local/share/gazebo/setup.bash
```

Create a catkin workspace:

```bash
mkdir -p asv_ws/src
cd asv_ws
catkin init
```

Configure catkin:

```bash
mkdir -p asv_ws/src
cd asv_ws
catkin init
```

### Clone and build the package

Clone the ASV Simulator repository:

```bash
cd src
git clone ssh://rhys@diskstation.local:/volume1/git/asv_sim.git
```

Compile the packages:

```bash
catkin build
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

![Anemometer World](https://github.com/srmainwaring/asv_sim/wiki/images/anemometer_world_falling.jpg)

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

## Acknowledgments


