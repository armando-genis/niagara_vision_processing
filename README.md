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

## Install for usage
OpenCV 4.5.4+
```bash
sudo apt-get install ros-foxy-vision-msgs
```



OPEN CV VERSION 4.7.0-dev
OpenCV 4.5.4+