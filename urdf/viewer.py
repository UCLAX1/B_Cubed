import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("bb8_car/urdf/bb8_hl.xml")
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

# Parameters
hang_duration = 10.0  # seconds to float before falling
start_time = time.time()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        mujoco.mj_step(model, data)
        viewer.sync()