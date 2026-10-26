#!/usr/bin/env python2
"""Isolated startup test; never connects to the chassis ROS master."""
from __future__ import print_function

import math
import os
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpclib


def main():
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    env = os.environ.copy()
    env.pop("ROS_HOSTNAME", None)
    env["ROS_IP"] = "127.0.0.1"
    env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    env["ROS_HOME"] = tempfile.mkdtemp(prefix="ranger-nav-test-")
    os.environ.update(env)
    children = []
    log_path = os.path.join(env["ROS_HOME"], "startup.log")
    log = open(log_path, "w")
    print("Isolated test log:", log_path)
    try:
        children.append(subprocess.Popen(
            ["roscore", "-p", str(port)], env=env, stdout=log,
            stderr=subprocess.STDOUT, preexec_fn=os.setsid))
        master = xmlrpclib.ServerProxy(env["ROS_MASTER_URI"])
        deadline = time.time() + 20
        while True:
            try:
                assert master.getPid("/navigation_smoke")[0] == 1
                break
            except Exception:
                if time.time() > deadline:
                    raise RuntimeError("Isolated master did not start")
                time.sleep(0.2)

        import rospy
        import tf
        import tf2_ros
        from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import LaserScan

        rospy.init_node("navigation_smoke", anonymous=True)
        scan_pub = rospy.Publisher("/helios/scan_filtered", LaserScan, queue_size=1)
        odom_pub = rospy.Publisher("/ranger_base_node/odom", Odometry, queue_size=1)
        pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        dynamic_tf = tf.TransformBroadcaster()
        static_tf = tf2_ros.StaticTransformBroadcaster()
        lidar = TransformStamped()
        lidar.header.stamp = rospy.Time.now()
        lidar.header.frame_id = "base_link"
        lidar.child_frame_id = "rslidar"
        lidar.transform.translation.x = 0.2767
        lidar.transform.translation.z = 0.661
        lidar.transform.rotation.z = 1.0
        static_tf.sendTransform(lidar)
        received = set()

        def record(msg, topic):
            if msg.data:
                received.add(topic)

        topics = ["/map", "/move_base/global_costmap/costmap",
                  "/move_base/local_costmap/costmap"]
        subscribers = [rospy.Subscriber(t, OccupancyGrid, record, callback_args=t)
                       for t in topics]
        children.append(subprocess.Popen(
            ["roslaunch", "ranger_navigation", "navigation.launch",
             "start_scan:=false", "rviz:=false"], env=env, stdout=log,
            stderr=subprocess.STDOUT, preexec_fn=os.setsid))
        deadline = time.time() + 45
        pose_sent = False
        while time.time() < deadline and not rospy.is_shutdown():
            now = rospy.Time.now()
            dynamic_tf.sendTransform((0, 0, 0), (0, 0, 0, 1),
                                     now, "base_link", "odom")
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1
            odom_pub.publish(odom)
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = "rslidar"
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.scan_time = 0.1
            scan.range_min = 0.5
            scan.range_max = 20
            scan.ranges = [5.0] * 360
            scan_pub.publish(scan)
            if not pose_sent and pose_pub.get_num_connections():
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = now
                pose.pose.pose.orientation.w = 1
                pose.pose.covariance[0] = 0.25
                pose.pose.covariance[7] = 0.25
                pose.pose.covariance[35] = 0.07
                pose_pub.publish(pose)
                pose_sent = True
            if all(t in received for t in topics):
                rospy.wait_for_service("/move_base/make_plan", timeout=5)
                break
            time.sleep(0.1)
        assert all(t in received for t in topics), ("Missing maps", received)
        code, _, state = master.getSystemState("/navigation_smoke")
        assert code == 1
        publishers = dict(state[0])
        assert "/move_base" in publishers.get("/navigation/cmd_vel", [])
        assert "/move_base" not in publishers.get("/cmd_vel", [])
        assert rospy.get_param("/move_base/DWAPlannerROS/max_vel_x") == 0.20
        assert rospy.get_param("/move_base/oscillation_timeout") == 60.0
        assert rospy.get_param("/move_base/oscillation_distance") == 0.15
        assert rospy.get_param("/move_base/recovery_behavior_enabled") is False
        assert rospy.get_param("/move_base/DWAPlannerROS/odom_topic") == "/ranger_base_node/odom"
        print("PASS: saved map, AMCL TF, costmaps, planner service and preview isolation")
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in reversed(children):
            deadline = time.time() + 12
            while child.poll() is None and time.time() < deadline:
                time.sleep(0.1)
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
            child.wait()
        log.close()


if __name__ == "__main__":
    main()
