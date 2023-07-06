gnome-terminal -- bash -c "source install/setup.sh; ros2 run sample tf_broadcaster"

gnome-terminal -- bash -c "source install/setup.sh; while true; do pgrep -f 'ros2 run sample tf_broadcaster' && break; done; rviz2 -d ~/.rviz2/pointcloud2_config.rviz"
