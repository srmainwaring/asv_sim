# Gazebo Tutorials

## Gazebo Rendering Resources

On macOS Gazebo is installed to:

- `/usr/local/bin/gazebo`.

The Gazebo environment is set by sourcing the script:

```bash
source /usr/local/share/gazebo-9/setup.sh
```

This exports the following environment variables:

- `GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org`
- `GAZEBO_RESOURCE_PATH=/usr/local/share/gazebo-9`
- `GAZEBO_PLUGIN_PATH=/usr/local/share/gazebo-9/plugins`
- `GAZEBO_MODEL_PATH=/usr/local/share/gazebo-9/models`

The Gazebo plugin libraries are located at:

- `/usr/local/lib/gazebo-9/plugins`

The Gazebo resource directory is organised as follows:

```bash
+ /usr/local/share/gazebo-9
  + media
    + audio
      | *.mp3
      | *.ogg
      | *.wav
    + dem
      | *.tif
    + fonts
    + gui
    + materials
      + programs
        | *.frag
        | *.glsl
        | *.vert
      + scripts
        | *.compositor
        | *.program
        | *.material
      + textures
        | *.png
    + models
      | *.dae
    + rtshaderlib
      | *.glsl
    + skyx
      | *.fragment
      | *.hlsl
      | *.material
      | *.png
      | *.vertex
  + models
    + sun
      | model.config
      | model.sdf
  + worlds
    | *.world
```
