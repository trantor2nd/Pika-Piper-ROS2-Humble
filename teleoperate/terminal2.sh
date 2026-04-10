export PYTHON_EXECUTABLE=/home/hsb/miniforge3/envs/py310/bin/python
export PYTHONPATH=/home/hsb/miniforge3/envs/py310/lib/python3.10/site-packages:$PYTHONPATH
source /home/data/Project/pika_ros/install/setup.bash
ros2 launch pika_remote_piper teleop_rand_single_piper.launch.py