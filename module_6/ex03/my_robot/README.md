# my_robot
 ```bash
   colcon build
  . ~/ws/install/setup.sh
  ros2 launch robot_bringup diff_drive.launch.py
 ```
Check (another terminal)
```bash
 gz topic -e -t /imu
 ```
