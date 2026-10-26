# Ranger saved-map navigation

Use the Ranger Mini 3.0 chassis driver on this Jetson:

```bash
roslaunch ranger_bringup ranger_mini_v3.launch publish_odom_tf:=true
```

Keep this chassis node and the existing lidar driver running. Do not use the
Mini V1 launch for this vehicle.
Stop `gmapping.launch` before starting navigation: AMCL, not gmapping, must
publish `map -> odom`. The launch includes the scan converter, footprint
filter and base-to-lidar static TF. Do not start a second copy of these nodes.

Source `/opt/ros/melodic/setup.bash` and the Ranger workspace in each terminal.

## Preview

```bash
roslaunch ranger_navigation navigation.launch
```

Use RViz **2D Pose Estimate** to set the robot's current map position and
heading. Confirm scans align with the saved map before selecting a **2D Nav
Goal**. Preview publishes speed commands only on `/navigation/cmd_vel`.
It does not connect move_base to the chassis; other running controllers can
still move the robot.

## Hardware trial

Stop the preview launch. Check `rostopic info /cmd_vel` and stop keyboard,
joystick and other command publishers in their original terminals before
allowing navigation to drive. Ensure only the intended chassis driver
receives velocity commands; legacy `/base_node` must not control the same CAN
interface at the same time. Put the chassis in commanded/CAN mode according
to its manual, with an operator ready at the emergency stop.

```bash
roslaunch ranger_navigation navigation.launch enable_motion:=true
```

Initialize pose again, verify scan alignment, then try a nearby goal in open
space. Default speed limits are 0.20 m/s and 0.25 rad/s; no lateral or reverse
commands or automatic recovery maneuvers are configured. Cancel a goal with:

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
```

Canceling is not an emergency stop. Use the physical stop if behavior is
unexpected. Verify commanded rotation direction and speed separately from
the previously corrected feedback direction. DWA assumes planar unicycle
motion; Ranger switches between steering and spin modes, so physical
trajectory tracking and mode-transition behavior still require testing.

The scan retains only a horizontal height band and has a 0.5 m minimum range.
It cannot guarantee detection of low obstacles, drop-offs or close objects.
This setup is an initial indoor navigation configuration, not a safety system.

## Map and scan options

`map_file:=/absolute/path/map.yaml` loads another saved map. With
`start_scan:=false`, separately provide `/helios/scan_filtered` and the lidar
static TF. The default odometry topic is `/ranger_base_node/odom`.
