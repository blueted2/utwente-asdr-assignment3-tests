

## Build and run nodes:
```
colcon build
. install/setup.bash
. /home/pi/ros2-xenomai/install/local_setup.sh
ros2 launch launch/closed_loop.yml
```
(this also build the xenomai controller however the executable is not installed)

## Build and run Xenomai controller:
```
cd xenomai-ros2-framework
cmake . -B build
make -C build

# give permissions to the rtp ports 10 and 15 needed by the controller
sudo chmod 777 /dev/rtp10
sudo chmod 777 /dev/rtp15

sudo ./build/jiwy # needs sudo rights
```

## Build and run unit test program (from project root):
```
gcc xenomai-ros2-framework/cvt_unit_test.c -o cvt_unit_test && ./cvt_unit_test
```