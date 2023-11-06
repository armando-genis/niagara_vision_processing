# niagara_vision_processing
 

  ## Set up ROS2
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
```

# Yolo version 5 - cpp
```bash
colcon build --packages-select yolov5_cpp
source install/setup.bash
ros2 launch yolov5_cpp yolov5cpp.launch.py
```
# Yolo version 5 - Python
```bash
colcon build --packages-select yolov5_niagara
source install/setup.bash
ros2 launch yolov5_niagara yolov5.launch.py
```
# Yolo version 8 - Python
```bash
colcon build --packages-select yolov8_niagara
source install/setup.bash
ros2 launch yolov8_niagara yolov8.launch.py
```

# Points Cloud Detector
```bash
colcon build --packages-select pointscloud_detector
source install/setup.bash
ros2 launch pointscloud_detector pcloudsDetec.launch.py
```

# Voxel Grid Filter
```bash
colcon build --packages-select voxel_grid_filter
source install/setup.bash
ros2 launch voxel_grid_filter filter.launch.py
```

## Install for usage
OpenCV 4.5.4+
PCL 1.10.0+

```bash
sudo apt-get install libpcl-dev
sudo apt-get install ros-foxy-vision-msgs
sudo apt-get install ros-foxy-image-geometry

```

## To do
From the v4 make that the function cloud2TransformedCloud and msg2TransformedCloud do no depends on _cam_model

## Links

 - [Message Filter](https://docs.ros.org/en/iron/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html#build)


