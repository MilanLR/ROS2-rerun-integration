#!/bin/bash
colcon build --symlink-install && . install/setup.bash && ros2 run rerun_visualization rerun_node
