# my_robot
colcon build
. ~/ws/install/setup.sh
ros2 launch robot_bringup diff_drive.launch.py
"gz topic -e -t /imu" - write it to check (another terminal)

