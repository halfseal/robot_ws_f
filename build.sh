rm -rf build/
rm -rf install/
rm -rf log

if [ "$1" == "remove" ] || [ "$1" == "rm" ]; then
        exit 0
fi

colcon build --symlink-install
