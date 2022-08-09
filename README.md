# Point One Navigation GPS Driver

This is a C++ ROS2 driver for [Point One Navigation](https://pointonenav.com) GNSS / INS Devices. 

> Less than 10cm location accuracy is accomplished by the Point One Navigation [FusionEngine](https://pointonenav.com/fusionengine) software.

### Getting Started

##### Install driver dependencies:
```bash
mkdir -p fusion_engine_ws/src
cd fusion_engine_ws/src
git clone https://github.com/airacingtech/fusion_engine_ros_driver.git
sudo apt update
sudo rosdep update
cd ../..
rosdep install -y --ignore-src --from-paths src/
```

##### Run driver node:

* Edit the parameters in `param/fusion_engine_ros_driver.param.yaml`.
* Optional: Remap output topic names in `launch/fusion_engine_ros_driver.launch.py`.

```bash
cd fusion_engine_ws
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
ros2 launch fusion_engine_ros_driver fusion_engine_ros_driver.launch.py
```
