#!/usr/bin/env python2
"""Expand launch parameters offline; never start a node or connect to CAN."""
from __future__ import print_function

import os
import subprocess

import rospkg

import yaml


def main():
    packages = rospkg.RosPack()
    for package, launch in [
            ("ranger_bringup", "ranger.launch"),
            ("ranger_bringup", "ranger_mini_v3.launch"),
            ("ranger_base", "ranger.launch"),
            ("ranger_base", "ranger_mini_v3.launch"),
            ("ranger_base", "include/ranger_robot_base.launch")]:
        path = os.path.join(packages.get_path(package), "launch", launch)
        output = subprocess.check_output([
            "roslaunch", "--dump-params", path,
            "publish_odom_tf:=true"])
        params = yaml.safe_load(output)
        prefix = "/ranger_base_node/"
        assert params[prefix + "robot_model"] == "ranger_mini_v3"
        assert params[prefix + "publish_odom_tf"] is True
        assert params[prefix + "port_name"] == "can0"
        assert params[prefix + "update_rate"] == 50
        assert params[prefix + "command_timeout"] == 0.6
        assert params[prefix + "odom_topic_name"] == "odom"
        print("PASS:", package, launch)


if __name__ == "__main__":
    main()
