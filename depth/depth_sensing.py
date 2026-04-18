import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import argparse

use_gpu = gl.GPU_ACCELERATION_AVAILABLE
mem_type = sl.MEM.GPU if use_gpu else sl.MEM.CPU

init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.NEURAL_LIGHT,
                         coordinate_units=sl.UNIT.METER,
                         coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
zed = sl.Camera()
status = zed.open(init)

if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()

res = sl.Resolution()
res.width = -1
res.height = -1

point_cloud = sl.Mat()
zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, mem_type, res)
res = point_cloud.get_resolution()

# Create OpenGL viewer
viewer = gl.GLViewer()
viewer.init(1, sys.argv, res)

while viewer.is_available():
    if zed.grab() <= sl.ERROR_CODE.SUCCESS:
        # Retrieve point cloud data using the optimal memory type (GPU if CuPy available)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, mem_type, res)
        viewer.updateData(point_cloud)
        if viewer.save_data:
            # For saving, we take CPU memory regardless of processing type
            point_cloud_to_save = sl.Mat()
            zed.retrieve_measure(point_cloud_to_save, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
            err = point_cloud_to_save.write('Pointcloud.ply')
            if (err == sl.ERROR_CODE.SUCCESS):
                print("Current .ply file saving succeed")
            else:
                print("Current .ply file failed")
            viewer.save_data = False
viewer.exit()
zed.close()