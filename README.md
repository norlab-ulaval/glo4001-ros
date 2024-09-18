# GLO-4001 ROS

```bash
git submodule update --init --recursive
```

## udev rules

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Running

```bash
g
./symlink_build.sh

ros2 launch rover_launchers rover_base_launch.xml
```
