# GLO-4001 ROS

```bash
git submodule update --init --recursive
```

## Running

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
./symlink_build.sh

ros2 launch rover_launchers rover_base_launch.xml
```
