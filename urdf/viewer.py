import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("D:/B_Cubed-2/bb8_car_thisone/urdf/bb8_car_thisone_converted.xml")
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

# Parameters
hang_duration = 10.0  # seconds to float before falling
start_time = time.time()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        mujoco.mj_step(model, data)
        viewer.sync()