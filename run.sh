#!/bin/zsh

PKG_DIR="$HOME/superbuild/src/slam_pkgs"
source "$PKG_DIR/install/setup.zsh"

MODE="${1:-mono_inertial}"
FNAME="${2:-cam_040626}"

if [[ "$MODE" == "mono" ]]; then
    NODE="mono_node_cpp"
    SETTINGS="G1"
elif [[ "$MODE" == "mono_inertial" ]]; then
    NODE="mono_inertial_node_cpp"
    SETTINGS="G1"
elif [[ "$MODE" == "stereo" ]]; then
    echo "Support for stereo is not available yet"
    exit 1
else
    echo "Unknown mode: $MODE. Use mono, mono_inertial, or stereo"
    exit 1
fi

ros2 run ros2_orb_slam3 $NODE --ros-args -p node_name_arg:=${NODE}_cpp -p settings_name:=$SETTINGS &

sleep 2

ros2 run ros2_orb_slam3 g1_driver_node.py -- --bag $PKG_DIR/src/data/$FNAME.bag --mode $MODE
