# Building This Workspace

The workspace targets Ubuntu 18.04 ARM64 and ROS Melodic on Jetson.
Install ROS Melodic and the dependencies listed in each package.xml before
building. Mapping and navigation additionally require gmapping, map_server,
pointcloud_to_laserscan, laser_filters, AMCL, move_base, global_planner and
dwa_local_planner. The RoboSense lidar driver lives in a separate workspace.

After cloning into `~/ranger_ws`:

```bash
source /opt/ros/melodic/setup.bash
cd ~/ranger_ws
catkin_init_workspace src
catkin_make -j2 -l2 \
  -DCMAKE_PREFIX_PATH=/opt/ros/melodic \
  -DPYTHON_EXECUTABLE=/usr/bin/python2 \
  -DCATKIN_ENABLE_TESTING=OFF
source devel/setup.bash --extend
```

Build products and the machine-specific catkin CMakeLists.txt symlink are
intentionally excluded from Git. `catkin_init_workspace src` recreates it.

The current ranger_ros, ugv_sdk and googletest sources are vendored directly
with local modifications, not Git submodules. Their original license files
remain alongside the sources. No submodule initialization is needed.

Use `README.md` for the local Ranger Mini 3.0 startup workflow. Maps under
`~/maps` and the lidar workspace are outside this repository.
