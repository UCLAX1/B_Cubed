colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
ros2 run person_tracking person

