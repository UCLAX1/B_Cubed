import mujoco

# Load URDF file (make sure path is correct and meshes are reachable)
model = mujoco.MjModel.from_xml_path("D:/Ballu_export/bb8_car/urdf/bb8_car.urdf")

# Save converted MJCF
mujoco.mj_saveLastXML("D:/Ballu_export/bb8_car/urdf/bb8_car_converted.xml", model)
