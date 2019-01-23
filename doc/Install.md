# ASV Common

Robot description and Gazebo worlds and launch files for an autonomous sailing vessel.

## Setup

These instructions suppose that you have created a workspace directory called
`~/asv` for the package in your home directory:

Source the package setup script:

```bash
cd ~/asv_ws
source ./devel/setup.bash
```

Update Gazebo environment variables

```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/asv_ws/devel/lib/
```


## Display the robot model in rviz

To display the robot model in `rviz` use `roslaunch` to run the `display.launch` script from the `urdf_tutorial` package.

```bash
roslaunch urdf_tutorial display.launch model:='$(find asv_description)/urdf/asv.urdf'
```

Refer to the [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials) for full details.

## Load the robot model in gazebo

Launch the gazebo world:

```bash
roslaunch asv_gazebo asv.launch
```

Spawn the robot model in gazebo:

```bash
rosrun gazebo_ros spawn_model -file `rospack find asv_description`/urdf/asv.urdf -urdf -model asv
```
