colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers "$(nproc)"
source install/setup.bash


gnome-terminal --title="hand tracking" -- bash -lc "
  source '/home/jetson-nano-x1/Documents/B_Cubed/ros2_ws/install/local_setup.bash'
  ros2 run person_tracking hands
  exec bash
"

gnome-terminal --title="person tracking" -- bash -lc "
  source '/home/jetson-nano-x1/Documents/B_Cubed/ros2_ws/install/local_setup.bash'
  ros2 run person_tracking person
  exec bash
"

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm


