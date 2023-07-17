# 2021_KMU_AUTORACE
## File Structure
```
├── README.md
├── ar_track_alvar - 라이브러리
├── launch
│   └── final.launch
├── obstacle_detector - 라이브러리
├── src
│   ├── Laser.py
│   ├── ObstacleDetector.py
│   ├── Pidcal.py
│   ├── RLineDetection.py
│   ├── Slidewindow.py
│   ├── Stop_Counter.py
│   ├── Stop_Detector.py
│   ├── Ultrasonic.py
│   └── main.py
└── vesc
    ├── vesc_ackermann
    │   └── launch
    │       ├── ackermann_to_vesc_node.launch
    │       ├── ackermann_to_vesc_nodelet.launch
    │       ├── vesc_to_odom_node.launch
    │       └── vesc_to_odom_nodelet.launch
    └── vesc_driver
        ├── launch
        │   ├── vesc_drive_xycar_motor.launch
        │   ├── vesc_driver_node.launch
        │   └── vesc_driver_nodelet.launch
        └── yaml
            └── vesc.yaml
```
