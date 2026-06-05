#!/bin/zsh

PKG_DIR="$HOME/superbuild/src/slam_pkgs"
source "$PKG_DIR/install/setup.zsh"

ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp -p settings_name:=G1 &

sleep 2
FNAME="${1:-cam_040626}"

ros2 run ros2_orb_slam3 g1_driver_node.py -- --bag $PKG_DIR/src/data/$FNAME.bag --mode mono