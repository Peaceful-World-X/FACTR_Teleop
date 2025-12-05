#!/bin/bash

# Franky terminal
gnome-terminal -- bash -c "
    echo '[Franky] Starting python 3.11 program...';
    cd /home/cytoderm/projects/FACTR_Teleop;
    source ~/anaconda3/etc/profile.d/conda.sh;
    conda activate franka;
    /home/cytoderm/anaconda3/envs/franka/bin/python3.11 /home/cytoderm/projects/FACTR_Teleop/franka_factr_bridge_franky.py;
    exec bash
"
sleep 2s
# ROS2 terminal
gnome-terminal -- bash -c "
    echo '[ROS2] Starting node...';
    cd /home/cytoderm/projects/FACTR_Teleop;
    source .venv/bin/activate;
    source install/local_setup.bash;
    source install/setup.bash;
    ros2 launch factr_teleop teleop_system.launch.py;
    exec bash
"