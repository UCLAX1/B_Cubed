#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class ZedDepthViewer(Node):
    """
    Subscribe to ZED depth (sensor_msgs/Image) and show a depth view window via OpenCV.

    Encodings handled:
      - 32FC1: float32 depth (typically meters)
      - 16UC1 / mono16: uint16 depth (often millimeters; configurable)

    Display:
      - converts depth to meters
      - clips to [min_depth, max_depth]
      - normalizes and applies colormap
      - shows window, click to print depth at pixel
    """

    def __init__(self):
        super().__init__("zed_depth_viewer")

        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("print_every", 30)
        self.declare_parameter("assume_16u_mm", True)

        # Display params
        self.declare_parameter("min_depth", 0.3)   # meters
        self.declare_parameter("max_depth", 5.0)   # meters
        self.declare_parameter("colormap", "turbo")  # turbo|jet|viridis|magma|inferno|plasma
        self.declare_parameter("window_name", "ZED Depth")

        self.depth_topic = self.get_parameter("depth_topic").value
        self.print_every = int(self.get_parameter("print_every").value)
        self.assume_16u_mm = bool(self.get_parameter("assume_16u_mm").value)

        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.colormap_name = str(self.get_parameter("colormap").value).lower()
        self.window_name = str(self.get_parameter("window_name").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().warning(
                "cv_bridge not found. Will decode Image via numpy fallback (supports 32FC1/16UC1)."
            )

        self.frame_count = 0
        self.last_depth_m = None  # last depth frame in meters

        self.sub = self.create_subscription(Image, self.depth_topic, self.cb_depth, qos)
        self.get_logger().info(f"Subscribing to: {self.depth_topic}")

        # OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        self.get_logger().info(
            f"Depth view: clip [{self.min_depth}m, {self.max_depth}m], "
            f"colormap={self.colormap_name}. "
            "Press 'q' or ESC to quit."
        )

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.last_depth_m is None:
            return
        h, w = self.last_depth_m.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            return
        d = float(self.last_depth_m[y, x])
        if not np.isfinite(d) or d <= 0.0:
            self.get_logger().info(f"Clicked ({x},{y}): invalid depth")
        else:
            self.get_logger().info(f"Clicked ({x},{y}): {d:.3f} m")

    def cb_depth(self, msg: Image):
        self.frame_count += 1

        depth_m = self._image_to_depth_meters(msg)
        self.last_depth_m = depth_m

        # Stats
        valid = depth_m[np.isfinite(depth_m)]
        valid = valid[valid > 0.0]
        if valid.size > 0 and (self.frame_count % self.print_every == 0):
            self.get_logger().info(
                f"Depth stats (m): min={float(valid.min()):.3f}, mean={float(valid.mean()):.3f}, "
                f"max={float(valid.max()):.3f} ({msg.width}x{msg.height}, enc={msg.encoding})"
            )

        # Display
        view = self._make_depth_view(depth_m)
        cv2.imshow(self.window_name, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):  # ESC or q
            # Stop spinning cleanly
            rclpy.shutdown()

    def blur_depth_image(self, depth_image, h, w):
        ratio = 100
        kernel_width = max(1, w // ratio)
        kernel_height = max(1, h // ratio)

        kernel = np.ones((kernel_height, kernel_width), np.float32)
        kernel /= (kernel_width * kernel_height)

        # filter2D will not behave nicely with NaNs, so temporarily replace invalids
        valid_mask = np.isfinite(depth_image) & (depth_image > 0.0)
        filled = depth_image.copy()
        filled[~valid_mask] = 0.0

        blurred_sum = cv2.filter2D(filled, -1, kernel)
        blurred_count = cv2.filter2D(valid_mask.astype(np.float32), -1, kernel)

        out = np.full_like(filled, np.nan, dtype=np.float32)
        good = blurred_count > 1e-6
        out[good] = blurred_sum[good] / blurred_count[good]

        return out

    def blur_depth_image_gpu(self, depth_image, h, w):
        ratio = 100
        kernel_width = max(1, w // ratio)
        kernel_height = max(1, h // ratio)

        kernel = np.ones((kernel_height, kernel_width), np.float32)
        kernel /= (kernel_width * kernel_height)

        valid_mask = (np.isfinite(depth_image) & (depth_image > 0.0)).astype(np.float32)
        filled = depth_image.copy().astype(np.float32)
        filled[~np.isfinite(filled)] = 0.0
        filled[filled <= 0.0] = 0.0

        gpu_image = cv2.cuda_GpuMat()
        gpu_mask = cv2.cuda_GpuMat()
        gpu_image.upload(filled)
        gpu_mask.upload(valid_mask)

        gpu_filter = cv2.cuda.createLinearFilter(cv2.CV_32F, cv2.CV_32F, kernel)

        gpu_blurred_sum = gpu_filter.apply(gpu_image)
        gpu_blurred_count = gpu_filter.apply(gpu_mask)

        blurred_sum = gpu_blurred_sum.download()
        blurred_count = gpu_blurred_count.download()

        out = np.full_like(filled, np.nan, dtype=np.float32)
        good = blurred_count > 1e-6
        out[good] = blurred_sum[good] / blurred_count[good]

        return out

    def get_cell_data(self, depth_image, grid_size):
        h, w = depth_image.shape

        depth_image = depth_image.astype(np.float32, copy=False)
        valid_mask = np.isfinite(depth_image) & (depth_image > 0.0)

        if not np.any(valid_mask):
            return np.full(grid_size, np.inf, dtype=np.float32)

        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                blurred_depth_image = self.blur_depth_image_gpu(depth_image, h, w)
            else:
                blurred_depth_image = self.blur_depth_image(depth_image, h, w)
        except Exception:
            blurred_depth_image = self.blur_depth_image(depth_image, h, w)

        cell_h, cell_w = h // grid_size[0], w // grid_size[1]
        cell_averages = np.full(grid_size, np.inf, dtype=np.float32)

        for i in range(grid_size[0]):
            for j in range(grid_size[1]):
                row_start = i * cell_h
                row_end = h if i == grid_size[0] - 1 else (i + 1) * cell_h
                col_start = j * cell_w
                col_end = w if j == grid_size[1] - 1 else (j + 1) * cell_w

                cell = blurred_depth_image[row_start:row_end, col_start:col_end]
                valid_cell = cell[np.isfinite(cell) & (cell > 0.0)]

                if valid_cell.size > 0:
                    cell_averages[i, j] = float(np.mean(valid_cell))

        return cell_averages

    def _make_depth_view(self, depth_m: np.ndarray) -> np.ndarray:
        # Clip and normalize to 0..255
        d = depth_m.copy()
        d[~np.isfinite(d)] = 0.0
        d[d < self.min_depth] = self.min_depth
        d[d > self.max_depth] = self.max_depth

        # Normalize: nearer -> brighter (invert if you prefer)
        norm = (d - self.min_depth) / max(1e-6, (self.max_depth - self.min_depth))
        norm = np.clip(norm, 0.0, 1.0)
        gray8 = (norm * 255.0).astype(np.uint8)

        cmap = self._colormap_code(self.colormap_name)
        color = cv2.applyColorMap(gray8, cmap)

        # Add a small legend text
        cv2.putText(
            color,
            f"{self.min_depth:.2f}m .. {self.max_depth:.2f}m",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return color

    def _colormap_code(self, name: str) -> int:
        # OpenCV has TURBO, JET; viridis/magma/inferno/plasma exist on newer builds.
        mapping = {
            "turbo": cv2.COLORMAP_TURBO,
            "jet": cv2.COLORMAP_JET,
        }
        # Optional maps if available:
        if hasattr(cv2, "COLORMAP_VIRIDIS"):
            mapping.update({
                "viridis": cv2.COLORMAP_VIRIDIS,
                "magma": cv2.COLORMAP_MAGMA,
                "inferno": cv2.COLORMAP_INFERNO,
                "plasma": cv2.COLORMAP_PLASMA,
            })
        return mapping.get(name, cv2.COLORMAP_TURBO)

    def _image_to_depth_meters(self, msg: Image) -> np.ndarray:
        enc = (msg.encoding or "").lower()

        if self.bridge is not None:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            arr = np.asarray(img)

            if enc in ("16uc1", "mono16"):
                if self.assume_16u_mm:
                    return arr.astype(np.float32) * 1e-3
                return arr.astype(np.float32)

            if enc == "32fc1":
                return arr.astype(np.float32)

            # Fallback
            return arr.astype(np.float32)

        # Numpy fallback (handles row padding via step)
        w, h, step = msg.width, msg.height, msg.step
        data_u8 = np.frombuffer(msg.data, dtype=np.uint8)

        if enc == "32fc1":
            row_bytes = w * 4
            out = np.empty((h, w), dtype=np.float32)
            for r in range(h):
                row = data_u8[r * step: r * step + row_bytes]
                out[r, :] = np.frombuffer(row.tobytes(), dtype=np.float32, count=w)
            return out

        if enc in ("16uc1", "mono16"):
            row_bytes = w * 2
            out16 = np.empty((h, w), dtype=np.uint16)
            for r in range(h):
                row = data_u8[r * step: r * step + row_bytes]
                out16[r, :] = np.frombuffer(row.tobytes(), dtype=np.uint16, count=w)
            out = out16.astype(np.float32)
            if self.assume_16u_mm:
                out *= 1e-3
            return out

        self.get_logger().warning_once(
            f"Unknown encoding '{msg.encoding}', interpreting as float32."
        )
        # This may be wrong, but gives something
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, -1)


def main():
    rclpy.init()
    node = ZedDepthViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
