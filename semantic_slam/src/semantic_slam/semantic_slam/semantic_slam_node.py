import math
import numpy as np
from typing import Type

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D,
    Detection3DArray, Detection3D
)
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

from message_filters import ApproximateTimeSynchronizer, Subscriber as MFSubscriber

# plugin API
from .detectors.base import DetectorBase, Det2D
from .detectors.yolo11 import YOLO11Detector


DETECTOR_REGISTRY = {
    'yolo11': YOLO11Detector,   # add more by importing and registering here
}


class SemanticSLAMNode(Node):
    def __init__(self):
        super().__init__('semantic_slam')

        # Frames and fusion params
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.camera_frame = self.declare_parameter('camera_frame', 'oakd_rgb_camera_optical_frame').value
        self.score_min = float(self.declare_parameter('fuser.score_min', 0.45).value)
        self.range_min = float(self.declare_parameter('fuser.range_min_m', 0.2).value)
        self.range_max = float(self.declare_parameter('fuser.range_max_m', 8.0).value)
        self.publish_markers = bool(self.declare_parameter('fuser.publish_markers', True).value)
        self.depth_scale = float(self.declare_parameter('fuser.depth_scale', 1.0).value)   # 32FC1 meters
        self.median_window = int(self.declare_parameter('fuser.median_window', 5).value)
        self.ground_clamp = bool(self.declare_parameter('fuser.ground_clamp', True).value)
        self.yaw_bias_deg = float(self.declare_parameter('fuser.yaw_bias_deg', 0.0).value)
        self.warn_stamp_delta_sec = float(self.declare_parameter('fuser.warn_stamp_delta_sec', 0.20).value)

        # TF tolerance and fallback
        self.tf_time_tolerance_sec = float(self.declare_parameter('tf_time_tolerance_sec', 0.05).value)
        self.tf_fallback_to_latest = bool(self.declare_parameter('tf_fallback_to_latest', True).value)
        self.tf_lookup_timeout_sec = float(self.declare_parameter('tf_lookup_timeout_sec', 0.10).value)

        # Detector selection and setup (pure Python plugin)
        det_type = self.declare_parameter('detector_type', 'yolo11').value
        det_cls: Type[DetectorBase] = DETECTOR_REGISTRY.get(det_type)
        if det_cls is None:
            available = ','.join(DETECTOR_REGISTRY.keys())
            raise RuntimeError(f"Unknown detector_type '{det_type}'. Available: {available}")
        self.detector: DetectorBase = det_cls()
        # Pull detector params and call setup
        det_params = dict(
            model=self.declare_parameter('detector.model', 'yolo11n.pt').value,
            conf=float(self.declare_parameter('detector.conf', 0.45).value),
            iou=float(self.declare_parameter('detector.iou', 0.45).value),
            max_det=int(self.declare_parameter('detector.max_det', 200).value),
            class_allowlist=self.declare_parameter(
                'detector.class_allowlist', [],
                ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
            ).get_parameter_value().string_array_value
        )
        self.detector.setup(**det_params)

        # QoS
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        # Subs
        self.bridge = CvBridge()
        self.sub_rgb = MFSubscriber(self, Image, '/oakd/rgb/preview/image_raw', qos_profile=sensor_qos)
        self.sub_depth = MFSubscriber(self, Image, '/oakd/rgb/preview/depth', qos_profile=sensor_qos)
        self.info_sub = self.create_subscription(CameraInfo, '/oakd/rgb/preview/camera_info',
                                                 self.cb_info, qos_profile=sensor_qos)

        # Sync RGB+Depth so detection and depth correspond
        sync_queue = int(self.declare_parameter('sync_queue', 10).value)
        sync_slop_sec = float(self.declare_parameter('sync_slop_sec', 0.10).value)
        self.ats = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth],
                                               queue_size=sync_queue, slop=sync_slop_sec)
        self.ats.registerCallback(self.cb_sync)

        # TF
        self.tfbuf = tf2_ros.Buffer()
        self.tflist = tf2_ros.TransformListener(self.tfbuf, self)

        # Pubs
        self.pub_det2d = self.create_publisher(Detection2DArray, '/detections', reliable_qos)
        self.pub_det3d = self.create_publisher(Detection3DArray, '/semantic_landmarks', reliable_qos)
        self.pub_mark = self.create_publisher(MarkerArray, '/semantic_markers', reliable_qos)

        # Cache intrinsics
        self.K = None
        self.cam_w = None
        self.cam_h = None

        # Warn flags
        self.warned_frame_mismatch = False
        self.warned_size_mismatch = False
        self.warned_stamp_delta = False

    # --- CameraInfo ---
    def cb_info(self, msg: CameraInfo):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
        self.cam_w = int(msg.width)
        self.cam_h = int(msg.height)

    # --- Utils ---
    def _scaled_intrinsics(self, fx, fy, cx, cy, depth_w, depth_h):
        if self.cam_w is None or self.cam_h is None:
            return fx, fy, cx, cy
        if depth_w == self.cam_w and depth_h == self.cam_h:
            return fx, fy, cx, cy
        sx = depth_w / float(self.cam_w)
        sy = depth_h / float(self.cam_h)
        if not self.warned_size_mismatch:
            self.get_logger().warn(
                f"Depth size {depth_w}x{depth_h} != CameraInfo {self.cam_w}x{self.cam_h}. Scaling intrinsics."
            )
            self.warned_size_mismatch = True
        return fx * sx, fy * sy, cx * sx, cy * sy

    def _median_depth_at(self, depth_m, u, v, win):
        H, W = depth_m.shape[:2]
        ui = int(round(u)); vi = int(round(v))
        if ui < 0 or vi < 0 or ui >= W or vi >= H:
            return float('nan')
        win = max(1, win | 1)
        half = win // 2
        u0 = max(0, ui - half); u1 = min(W, ui + half + 1)
        v0 = max(0, vi - half); v1 = min(H, vi + half + 1)
        patch = depth_m[v0:v1, u0:u1]
        good = patch[np.isfinite(patch)]
        if good.size == 0:
            return float('nan')
        return float(np.median(good))

    def _do_transform_with_tolerance(self, p_cam, stamp):
        tol = Duration(seconds=self.tf_time_tolerance_sec)
        try:
            tf = self.tfbuf.lookup_transform(
                self.map_frame, self.camera_frame, stamp,
                timeout=Duration(seconds=self.tf_lookup_timeout_sec)
            )
            return tf2_geometry_msgs.do_transform_point(p_cam, tf)
        except Exception:
            pass
        try:
            earlier = rclpy.time.Time.from_msg(stamp) - tol
            tf2 = self.tfbuf.lookup_transform(
                self.map_frame, self.camera_frame, earlier.to_msg(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec)
            )
            return tf2_geometry_msgs.do_transform_point(p_cam, tf2)
        except Exception:
            if self.tf_fallback_to_latest:
                tf_latest = self.tfbuf.lookup_transform(
                    self.map_frame, self.camera_frame, rclpy.time.Time().to_msg(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec)
                )
                return tf2_geometry_msgs.do_transform_point(p_cam, tf_latest)
            raise

    # --- Sync callback: RGB + Depth ---
    def cb_sync(self, rgb_msg: Image, depth_msg: Image):
        if self.K is None:
            return

        # Frame check
        if depth_msg.header.frame_id != self.camera_frame and not self.warned_frame_mismatch:
            self.get_logger().warn(
                f"Depth frame_id '{depth_msg.header.frame_id}' != camera_frame '{self.camera_frame}'. Depth likely not registered."
            )
            self.warned_frame_mismatch = True

        # Timestamp check
        dt = abs((rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec*1e-9) -
                 (depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec*1e-9))
        if dt > self.warn_stamp_delta_sec and not self.warned_stamp_delta:
            self.get_logger().warn(f"RGB vs depth time delta {dt:.3f}s exceeds {self.warn_stamp_delta_sec:.3f}s.")
            self.warned_stamp_delta = True

        # Convert images
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        # Depth to meters
        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) * self.depth_scale
        else:
            depth_m = depth_raw.astype(np.float32)
        H, W = depth_m.shape[:2]

        fx0, fy0, cx0, cy0 = self.K
        fx, fy, cx, cy = self._scaled_intrinsics(fx0, fy0, cx0, cy0, W, H)

        # Run detector (pure Python)
        dets = self.detector.detect(bgr)

        # Publish 2D detections
        det2d = Detection2DArray()
        det2d.header.stamp = rgb_msg.header.stamp
        det2d.header.frame_id = self.camera_frame
        for d in dets:
            if d.score < self.score_min:
                continue
            msg = Detection2D()
            msg.header = det2d.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.cls
            hyp.hypothesis.score = float(d.score)
            msg.results.append(hyp)
            bb = BoundingBox2D()
            bb.center.position.x = d.box.cx
            bb.center.position.y = d.box.cy
            bb.size_x = d.box.w
            bb.size_y = d.box.h
            msg.bbox = bb
            det2d.detections.append(msg)
        self.pub_det2d.publish(det2d)

        # Fuse to 3D using depth timestamp for TF
        det3d = Detection3DArray()
        det3d.header.stamp = depth_msg.header.stamp
        det3d.header.frame_id = self.map_frame

        markers = MarkerArray()
        mid = 0

        for d in det2d.detections:
            if not d.results:
                continue
            hyp = max(d.results, key=lambda r: r.hypothesis.score)
            if hyp.hypothesis.score < self.score_min:
                continue
            cls = hyp.hypothesis.class_id

            # Pixel at bottom-center of 2D bbox
            u = float(d.bbox.center.position.x)
            v = float(d.bbox.center.position.y + 0.5 * d.bbox.size_y)
            if u < 0 or v < 0 or u >= W or v >= H:
                continue

            Z = self._median_depth_at(depth_m, u, v, self.median_window)
            if not np.isfinite(Z) or Z < self.range_min or Z > self.range_max:
                continue

            # Back-project center
            Xc = (u - cx) * Z / fx
            Yc = (v - cy) * Z / fy

            # Compute 3D size from 2D bbox and depth
            # width_m derived from pixel width at depth Z, height_m from pixel height
            width_m = max(1e-3, float(d.bbox.size_x) * Z / fx)
            height_m = max(1e-3, float(d.bbox.size_y) * Z / fy)
            depth_thickness_m = max(1e-3, 0.25 * (width_m + height_m) * 0.5)  # heuristic

            p_cam = PointStamped()
            p_cam.header.stamp = depth_msg.header.stamp
            p_cam.header.frame_id = self.camera_frame
            p_cam.point.x = Xc
            p_cam.point.y = Yc
            p_cam.point.z = Z

            try:
                p_map = self._do_transform_with_tolerance(p_cam, depth_msg.header.stamp)
            except Exception as e:
                self.get_logger().warn(f'tf transform failed: {e}')
                continue

            if self.ground_clamp:
                p_map.point.z = 0.0

            if abs(self.yaw_bias_deg) > 1e-6:
                c = math.cos(math.radians(self.yaw_bias_deg))
                s = math.sin(math.radians(self.yaw_bias_deg))
                x, y = p_map.point.x, p_map.point.y
                p_map.point.x = c*x - s*y
                p_map.point.y = s*x + c*y

            d3 = Detection3D()
            d3.header = det3d.header
            d3.results.append(ObjectHypothesisWithPose(hypothesis=hyp.hypothesis))
            d3.bbox.center.position.x = p_map.point.x
            d3.bbox.center.position.y = p_map.point.y
            d3.bbox.center.position.z = p_map.point.z
            d3.bbox.size.x = width_m
            d3.bbox.size.y = height_m
            d3.bbox.size.z = depth_thickness_m
            det3d.detections.append(d3)

            if self.publish_markers:
                # Text label
                mk = Marker()
                mk.header = det3d.header
                mk.ns = 'sem'
                mk.id = mid; mid += 1
                mk.type = Marker.TEXT_VIEW_FACING
                mk.action = Marker.ADD
                mk.pose.position = d3.bbox.center.position
                mk.scale.z = 0.25
                mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 0.0; mk.color.a = 1.0
                mk.lifetime.sec = 0
                mk.text = f'{cls} ({hyp.hypothesis.score:.2f})'
                markers.markers.append(mk)

                # 3D box with size taken from detection
                mb = Marker()
                mb.header = det3d.header
                mb.ns = 'sem_box'
                mb.id = mid; mid += 1
                mb.type = Marker.CUBE
                mb.action = Marker.ADD
                mb.pose.position = d3.bbox.center.position
                mb.pose.orientation.w = 1.0
                mb.scale.x = max(1e-3, d3.bbox.size.x)
                mb.scale.y = max(1e-3, d3.bbox.size.y)
                mb.scale.z = max(1e-3, d3.bbox.size.z)
                mb.color.r = 0.0; mb.color.g = 1.0; mb.color.b = 0.0; mb.color.a = 0.35
                mb.lifetime.sec = 0
                markers.markers.append(mb)

        self.pub_det3d.publish(det3d)
        if self.publish_markers and markers.markers:
            self.pub_mark.publish(markers)


def main():
    rclpy.init()
    rclpy.spin(SemanticSLAMNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
