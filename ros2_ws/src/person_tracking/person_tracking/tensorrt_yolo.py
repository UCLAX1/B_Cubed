import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda
import numpy as np
import cv2

class TRTModel:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.ERROR)
        trt.init_libnvinfer_plugins(self.logger, "")

        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # Input/output binding info
        self.input_idx = 0
        self.output_idx = 1

        # Assume input shape = (1,3,640,640)
        shape = self.engine.get_binding_shape(self.input_idx)
        self.h, self.w = shape[2], shape[3]

        self.input_host = np.empty(shape, dtype=np.float32)
        self.output_host = np.empty(self.engine.get_binding_shape(self.output_idx), dtype=np.float32)

        self.input_dev = cuda.mem_alloc(self.input_host.nbytes)
        self.output_dev = cuda.mem_alloc(self.output_host.nbytes)

    def preprocess(self, frame):
        img = cv2.resize(frame, (self.w, self.h))
        img = img[:, :, ::-1] / 255.0
        img = np.transpose(img, (2, 0, 1)).astype(np.float32)
        return np.expand_dims(img, axis=0)

    def infer(self, frame):
        inp = self.preprocess(frame)
        cuda.memcpy_htod(self.input_dev, inp)

        self.context.execute_v2([int(self.input_dev), int(self.output_dev)])

        cuda.memcpy_dtoh(self.output_host, self.output_dev)

        return self.output_host.reshape(-1, 84)  # YOLO11n = 84 numbers per box
