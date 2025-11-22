import mujoco

# Load URDF file (make sure path is correct and meshes are reachable)
model = mujoco.MjModel.from_xml_path("D:/B_Cubed-2/bb8_car_thisone/urdf/bb8_car_thisone.urdf")

# Save converted MJCF
mujoco.mj_saveLastXML("D:/B_Cubed-2/bb8_car_thisone/urdf/bb8_car_thisone_converted.xml", model)
