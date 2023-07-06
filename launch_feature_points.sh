rm -rf build/
rm -rf install/
rm -rf log

if [ "$1" == "remove" ] || [ "$1" == "rm" ]; then
        exit 0
fi

colcon build --symlink-install

#source install/setup.sh

gnome-terminal -- bash -c "source install/setup.sh; ros2 launch sample feature_points.launch.py"

gnome-terminal -- bash -c "source install/setup.sh; while true; do pgrep -f 'ros2 launch sample feature_points.launch.py' && break; done; ros2 launch sample build_map.launch.py; ros2 run teleop_twist_keyboard teleop_twist_keyboard"
