# Introduction

The packages is forked from [shilohc/map2gazebo](https://github.com/shilohc/map2gazebo).
Now support ROS 2 foxy and offline conversion.

# ROS package for creating Gazebo environments from 2D maps

Subscribes to your map topic and exports a mesh for use in Gazebo in the
destination folder you specify.  The mesh will have obstacles (tall boxes)
corresponding to all occupied map squares.  Requires a map to be publishing
somewhere, and will probably work better if the map is static.

## Arguments and parameters

Default parameters are specified in config/defaults.yaml; the location of the
YAML parameter file is also a launchfile argument.  To change the defaults,
make a new YAML parameter file, and run

```
ros2 launch map2gazebo map2gazebo.launch.py params_file:=params.yaml
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
 * `export_dir`: Directory of output mesh Default to current directory.

# Usage

## Installation

NOTE:

trimesh needs the following soft dependencies to export Collada (.dae) files.

Theoretically you can install these with `pip install trimesh[soft]` but this
failed for me, so I installed the needed ones myself.

1. Install the python dependencies with pip:

```bash
pip3 install --user trimesh
pip3 install --user numpy
pip3 install --user pycollada
pip3 install --user scipy
pip3 install --user networkx
pip3 install --user opencv-contrib-python 

```

2. Git clone map2gazebo and build package

```bash
mkdir -p map2gz_ros2_ws/src
cd map2gz_ros2_ws/src
git clone https://github.com/Adlink-ROS/map2gazebo.git -b foxy-devel
cd ..
colcon build --symlink-install
```

## Online conversion

After you launch SLAM applicatoin and make sure "/map" topic is published.

```bash
source install/local_setup.bash
ros2 launch map2gazebo map2gazebo.launch.py
```
Remember to turn off the node when the map is **completely done**

## Offline conversion

If you want to convert an existing map to stl model. 

please modified map_dir and export_dir to directory on your PC.

```bash
python3 src/map2gazebo/src/map2gazebo_offline.py --map_dir /path/to/map/mememan.pgm --export_dir /path/to/export_dir
```

# Example

Here we provide an usage example to show the converted model in Gazebo.

## Online conversion

* Run NeuronBot

```bash
# terminal 1
ros2 launch neuronbot2_gazebo  neuronbot2_world.launch.py
# terminal 2
ros2 launch neuronbot2_slam  gmapping_launch.py open_rviz:=true use_sim_time:=true
# terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* Do some SLAM...

* Convert the stl, which will saved under current folder.

```bash
ros2 launch map2gazebo map2gazebo.launch.py
```

* Run Gazebo

```bash
mv map.stl src/map2gazebo/models/map/meshes/
export GAZEBO_MODEL_PATH=$HOME/map2gz_ros2_ws/src/map2gazebo/models/
gazebo src/map2gazebo/worlds/map.sdf
```

## Offline conversion

* Create your map (or download our template)

```bash
cd ~/map2gz_ros2_ws
wget -cO - https://github.com/Adlink-ROS/neuronbot2/blob/foxy-devel/neuronbot2_nav/map/wg.pgm?raw=true > wg.pgm
wget https://raw.githubusercontent.com/Adlink-ROS/neuronbot2/foxy-devel/neuronbot2_nav/map/wg.yaml
```

* Convert the map to STL model
  - Note the output directory: Gazebo world will use the model here

```bash
python3 src/map2gazebo/map2gazebo/map2gazebo_offline.py --map_dir wg.pgm --export_dir src/map2gazebo/models/map/meshes
```

* Modify the model file: `src/map2gazebo/models/map/model.sdf`
  - Replace `map.stl` to `wg.stl` (or your stl name)

* Run Gazebo

```bash
export GAZEBO_MODEL_PATH=$HOME/map2gz_ros2_ws/src/map2gazebo/models/
gazebo src/map2gazebo/worlds/map.sdf
```
