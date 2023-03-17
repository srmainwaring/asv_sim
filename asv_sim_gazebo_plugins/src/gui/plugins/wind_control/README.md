# Wind Control GUI system plugin

This is a GUI system plugin for controlling the wind environment. It is based on the Gazebo Sim example `gz-sim/examples/plugin/gui_system_plugin` which demonstrates how these plugins can access entity and component updates coming from the server.

See `WindControl.hh` for more information.

## Build

From the root of the `asv_sim` repository, do the following to build the example:

```bash
$ cd asv_sim_gazebo_plugins/src/gui/plugin/wind_control
$ mkdir build
$ cd build
$ cmake ..
$ make
```

This will generate the `WindControl` library under `build`.

## Run

Add the library to the `GZ_GUI_PLUGIN_PATH`:

```bash
$ cd asv_sim_gazebo_plugins/src/gui/plugin/wind_control
$ export GZ_GUI_PLUGIN_PATH=$(pwd)/build
```

Then run a world, for example:

```
$ gz sim -v4 waves.sdf
```

From the GUI plugin menu on the top-right, choose "Wind Control".
