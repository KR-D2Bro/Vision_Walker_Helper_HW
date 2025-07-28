#!/usr/bin/env bash
source /home/dongjae/ros2_humble_venv/bin/activate
source /home/dongjae/ros2_humble_ws/install/setup.bash
source /home/dongjae/ros2_kart_ws/install/setup.bash
exec ros2 launch launch_pkg launch.py

