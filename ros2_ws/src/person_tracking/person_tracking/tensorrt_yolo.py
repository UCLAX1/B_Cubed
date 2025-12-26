import tensorrt as trt
import pycuda.autoinit  # creates a CUDA context
import pycuda.driver as cuda
import numpy as np
import cv2


class TRTModel:
    def __init__(self, engine_path: str, input_hw=(640, 640), batch_size: int = 1):
        """
        input_hw: (H, W) you want to run at (used if the engine input has dynamic dims)
        """
        self.logger = trt.Logger(trt.Logger.ERROR)
        trt.init_libnvinfer_plugins(self.logger, "")

        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine: {engine_path}")

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("Failed to create execution context")

        self.stream = cuda.Stream()

        # --- Discover I/O tensor names (TRT 10) or bindings (TRT 8) ---
        self._use_name_api = hasattr(self.engine, "num_io_tensors") and hasattr(self.engine, "get_tensor_name")

        if self._use_name_api:
            self.input_names, self.output_names = self._get_io_names_trt10()
            if len(self.input_names) != 1:
                raise RuntimeError(f"Expected exactly 1 input tensor, got: {self.input_names}")
            if len(self.output_names) < 1:
                raise RuntimeError("Expected at least 1 output tensor")

            self.input_name = self.input_names[0]
            self.output_name = self.output_names[0]  # adjust if you have multiple outputs

            # Set runtime input shape if dynamic
            in_shape = tuple(self.engine.get_tensor_shape(self.input_name))
            # in_shape usually like (1,3,640,640) or (-1,3,-1,-1)
            if any(d < 0 for d in in_shape):
                H, W = input_hw
                ok = self.context.set_input_shape(self.input_name, (batch_size, 3, H, W))
                if not ok:
                    raise RuntimeError(f"set_input_shape failed for {self.input_name}")

            # After setting input shape, output shapes become known
            self.in_shape = tuple(self.context.get_tensor_shape(self.input_name))
            self.out_shape = tuple(self.context.get_tensor_shape(self.output_name))

            self.in_dtype = trt.nptype(self.engine.get_tensor_dtype(self.input_name))
            self.out_dtype = trt.nptype(self.engine.get_tensor_dtype(self.output_name))

        else:
            # TRT 8-style fallback
            self.input_idx, self.output_idx = self._get_io_indices_trt8()
            self.in_shape = tuple(self.engine.get_binding_shape(self.input_idx))
            self.out_shape = tuple(self.engine.get_binding_shape(self.output_idx))

            self.in_dtype = trt.nptype(self.engine.get_binding_dtype(self.input_idx))
            self.out_dtype = trt.nptype(self.engine.get_binding_dtype(self.output_idx))

            if any(d < 0 for d in self.in_shape):
                # If you ever hit this, you need an optimization profile; keeping simple here.
                raise RuntimeError(f"Dynamic binding shapes not handled in TRT8 fallback: {self.in_shape}")

        # Expect NCHW
        if len(self.in_shape) != 4:
            raise RuntimeError(f"Expected 4D input (NCHW). Got: {self.in_shape}")

        _, c, h, w = self.in_shape
        if c != 3:
            raise RuntimeError(f"Expected 3-channel input. Got C={c} shape={self.in_shape}")
        self.h, self.w = int(h), int(w)

        # --- Allocate buffers ---
        self.input_host = cuda.pagelocked_empty(self.in_shape, dtype=self.in_dtype)
        self.output_host = cuda.pagelocked_empty(self.out_shape, dtype=self.out_dtype)

        self.input_dev = cuda.mem_alloc(self.input_host.nbytes)
        self.output_dev = cuda.mem_alloc(self.output_host.nbytes)

        # For TRT 10 name-based execution, we must bind addresses by tensor name.
        if self._use_name_api and hasattr(self.context, "set_tensor_address"):
            self.context.set_tensor_address(self.input_name, int(self.input_dev))
            self.context.set_tensor_address(self.output_name, int(self.output_dev))

    def _get_io_names_trt10(self):
        inputs, outputs = [], []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                inputs.append(name)
            elif mode == trt.TensorIOMode.OUTPUT:
                outputs.append(name)
        return inputs, outputs

    def _get_io_indices_trt8(self):
        input_idx = output_idx = None
        for i in range(self.engine.num_bindings):
            if self.engine.binding_is_input(i):
                input_idx = i
            else:
                output_idx = i
        if input_idx is None or output_idx is None:
            raise RuntimeError("Could not find input/output bindings in TRT8 fallback")
        return input_idx, output_idx

    def preprocess(self, frame):
        # frame is assumed BGR uint8 from OpenCV
        img = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_LINEAR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        img = np.expand_dims(img, axis=0)   # -> NCHW
        return img

    def infer(self, frame):
        inp = self.preprocess(frame)

        # Copy into pinned host, then HtoD async
        np.copyto(self.input_host, inp)
        cuda.memcpy_htod_async(self.input_dev, self.input_host, self.stream)

        # Execute
        if hasattr(self.context, "execute_async_v3"):
            # TRT 10 path: addresses already set via set_tensor_address()
            self.context.execute_async_v3(stream_handle=self.stream.handle)
        elif hasattr(self.context, "execute_async_v2"):
            # TRT 8-ish path
            bindings = [0] * self.engine.num_bindings
            bindings[self.input_idx] = int(self.input_dev)
            bindings[self.output_idx] = int(self.output_dev)
            self.context.execute_async_v2(bindings=bindings, stream_handle=self.stream.handle)
        else:
            # Very old fallback
            self.context.execute_v2([int(self.input_dev), int(self.output_dev)])

        # DtoH async + sync
        cuda.memcpy_dtoh_async(self.output_host, self.output_dev, self.stream)
        self.stream.synchronize()

        # Return raw output (shape depends on export)
        out = np.array(self.output_host, copy=True).reshape(self.out_shape)

        # Normalize common YOLO TRT output shapes to (N, 84)
        out = np.squeeze(out)  # drop batch dim if present

        # Cases:
        # (84, N)  -> transpose -> (N, 84)
        # (N, 84)  -> already good
        if out.ndim == 2 and out.shape[0] in (84, 85) and out.shape[1] > out.shape[0]:
            out = out.T

        return out