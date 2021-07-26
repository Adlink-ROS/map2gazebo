# ROS package for creating Gazebo environments from 2D maps

Subscribes to your map topic and exports a mesh for use in Gazebo in the
destination folder you specify.  The mesh will have obstacles (tall boxes)
corresponding to all occupied map squares.  Requires a map to be publishing
somewhere, and will probably work better if the map is static; I recommend
using `map_server` with a saved map.  

If run with default parameters, it will write the mesh to this package's
models/map/meshes folder.  You will then be able to run `gazebo_world.launch`
to launch Gazebo pre-populated with the map mesh.  

## Arguments and parameters

The export directory is specified as a launchfile argument.  Change it using
```
roslaunch map2gazebo map2gazebo.launch export_dir:=/path/to/export_dir
```
Note that if you change the export directory, `gazebo_world.launch` will not
work unmodified.

Default parameters are specified in config/defaults.yaml; the location of the
YAML parameter file is also a launchfile argument.  To change the defaults,
make a new YAML parameter file, and run
```
roslaunch map2gazebo map2gazebo.launch params_file:=/path/to/your/params.yaml
```
Alternatively you could just edit the default parameter file (not recommended,
but I can't stop you).

YAML parameters:
 * `map_topic`: Topic that the map to convert is being published on.
 * `mesh_type`: Can be "stl" or "dae"; sets whether to export the mesh as a stl
(default) or as a Collada file.  Note that dae files specify a color and that
by default this color is black, which you will likely want to edit as it makes
your world very hard to see.  If exporting as dae, you will also need to modify
`models/map/model.sdf` to specify `map.dae` instead of `map.stl` if you are
planning to use `gazebo_world.launch`.
 * `occupied_thresh`: Minimum threshold value for considering a cell occupied. 
Defaults to 1 (out of 100).  
 * `box_height`: Height of boxes in gazebo environment.  Defaults to 2m. 

The YAML file does not specify the export directory because it doesn't seem to
support a value with substitution args like
`"$(find map2gazebo)/models/map/meshes"`, which is the desired default value.
The roslaunch `<arg>` and `<param>` tags are therefore used, but unfortunately
the `<param>` tag overrides any non-default value loaded from the YAML file.
Therefore, don't use the YAML parameter file to set a non-default export
directory.  (Let me know, or better submit a PR, if you know of a more elegant
way to do this!)

## Installation

Clone the repo and install ROS dependencies with rosdep.  

Install the python dependencies with pip:
```
pip install --user trimesh
pip install --user numpy
```

trimesh needs the following soft dependencies to export Collada (.dae) files.
Theoretically you can install these with `pip install trimesh[soft]` but this
failed for me, so I installed the needed ones myself.
```
pip install --user pycollada
pip install --user scipy
pip install --user networkx
```

