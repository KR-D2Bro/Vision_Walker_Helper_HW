#!/usr/bin/env bash
source /home/dongjae/ros2_iron_venv/bin/activate
source /home/dongjae/ros2_iron_ws/install/setup.bash
source /home/dongjae/Vision_Walker_Helper_HW/ros2_kart_ws/install/setup.bash
exec ros2 launch launch_pkg launch.py

