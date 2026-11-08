source install/setup.bash
ros2 launch dog_bringup dog_bringup.launch.py map_root:="$PWD/data/maps" slam_output:="$PWD/data/slam" motion_config:="$PWD/data/config/motion.json" "$@"
