# 2023_KMU_Autonomous_team_AuTURBO_rookie
2023 KMU Autonomous Driving Contest 

## Prepare
### install package for Yolo
```
$ pip3 install torch==1.6.0+cpu torchvision==0.8.1 torchaudio --index-url https://download.pytorch.org/whl/cpu
$ cd ./yolov5_ros/src/yolov5
$ pip install -r requirements.txt
```

## Execute
### Execute using real-time sensor
```
$ roslaunch auturbo_rookie_bringup bringup.launch
$ rosrun auturbo_rookie_perception line_detector.py
```

### Execute bag file
```
$ roscore

$ cd <bag file location>
$ rosbag play <bag file>.bag -l

$ roslaunch yolov5_ros yolov5.launch

$ rosrun auturbo_rookie_perception line_detector.py
```