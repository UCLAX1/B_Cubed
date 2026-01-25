colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers "$(nproc)"
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
#ros2 run person_tracking person

