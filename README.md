# Local Ranger Mini 3.0 Setup

This Jetson controls a Ranger Mini 3.0. Use `ranger_mini_v3`, not the Mini V1
protocol adapter. Both generic `ranger.launch` entry points and the base
node's fallback model now default to V3.

## Chassis

Source the workspace in each terminal:

```bash
source /opt/ros/melodic/setup.bash
source ~/ranger_ws/devel/setup.bash --extend
```

After power-on, enable CAN once if `can0` is not already up at 500000 bit/s:

```bash
ip -details link show can0
rosrun ranger_bringup bringup_can2usb.bash
```

Stop previous chassis drivers and all velocity publishers before starting:

```bash
roslaunch ranger_bringup ranger_mini_v3.launch publish_odom_tf:=true
```

Confirm the running model:

```bash
rosparam get /ranger_base_node/robot_model
```

Expected: `ranger_mini_v3`. Odometry remains `/ranger_base_node/odom`, with
`odom -> base_link` TF. The existing 50 Hz refresh and 0.6-second command
timeout remain enabled. No launch here starts automatically at boot.

## Mapping And Navigation

Start the existing lidar driver separately. Mapping and navigation reuse
the running V3 chassis; neither starts another chassis driver.

```bash
roslaunch ranger_mapping gmapping.launch
```

Stop mapping before saved-map navigation. Preview does not send navigation
commands to the chassis:

```bash
roslaunch ranger_navigation navigation.launch enable_motion:=false
```

After checking localization and scan alignment, stop preview and all keyboard
or other velocity publishers. With a clear test area and emergency stop ready:

```bash
roslaunch ranger_navigation navigation.launch enable_motion:=true
```

V1/V2 named launch files and SDK adapters are retained for compatibility, not
for this vehicle. Earlier V1-only sign corrections do not affect V3.
