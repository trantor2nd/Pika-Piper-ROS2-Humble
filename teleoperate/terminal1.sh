cd /home/data/Project/pika_ros/src/PikaAnyArm/piper/piper_ros
source ./can_activate.sh can0 1000000
source /home/data/Project/pika_ros/install/setup.bash
cd /home/data/Project/pika_ros/scripts && bash start_sensor_gripper.bash