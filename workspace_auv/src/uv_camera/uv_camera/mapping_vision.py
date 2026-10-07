"""Down-stereo mapping frontend. Frames and YOLO masks stay inside uv_camera."""

from collections import deque
from bisect import bisect_right
import math
import threading
import time

import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import MappingObservation, MappingObservationArray, PoseInfo

from .object_localizer import StereoCalibration


def _seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rpy_matrix(roll, pitch, yaw):
    roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


class MappingVision:
    """Emit compact, capture-timestamped world measurements per stereo pair."""

    def __init__(self, node, sim_mode):
        self.node = node
        self.sim_mode = bool(sim_mode)
        self.lock = threading.Lock()
        self.poses = deque(maxlen=300)
        self.infos = {}
        self.calibration = None
        self.maps = None
        self.size = None
        self.last_error_log = 0.0
        self.debug_frames = {}
        self.debug_stamp = 0.0
        self.debug_lock = threading.Lock()
        self.pub = node.create_publisher(
            MappingObservationArray, '/perception/mapping/observations',
            qos_profile_sensor_data)
        self.pose_sub = node.create_subscription(
            PoseInfo, '/basic_motion/pose_info', self._pose_cb,
            qos_profile_sensor_data)
        if self.sim_mode:
            self.info_subs = [node.create_subscription(
                CameraInfo, f'/sim/down_cam/{side}/camera_info',
                lambda msg, side=side: self._info_cb(side, msg),
                qos_profile_sensor_data) for side in ('left', 'right')]
        else:
            path = str(node.get_parameter('mapping_calibration_file').value).strip()
            if not path:
                from .down_calibration import real_down_calibration_path
                path = real_down_calibration_path()
            self.calibration = StereoCalibration.load('down_mapping', path)
            node.get_logger().info(f'建图视觉标定已加载：{path}')

        self.left_translation = np.array(
            node.get_parameter('mapping_left_translation').value, dtype=float)
        self.right_translation = np.array(
            node.get_parameter('mapping_right_translation').value, dtype=float)
        self.camera_rotation = np.array(
            node.get_parameter('mapping_camera_rotation').value,
            dtype=float).reshape(3, 3)
        self.min_depth = float(node.get_parameter('mapping_min_depth_m').value)
        self.max_depth = float(node.get_parameter('mapping_max_depth_m').value)
        self.min_points = int(node.get_parameter('mapping_min_depth_points').value)
        self.bin_m = float(node.get_parameter('mapping_depth_bin_m').value)
        self.peak_ratio = float(node.get_parameter('mapping_depth_peak_ratio').value)
        self.cone_height_m = float(node.get_parameter('mapping_cone_height_m').value)
        self.min_confidence = float(node.get_parameter('mapping_min_confidence').value)
        self.max_pose_age = float(node.get_parameter('mapping_max_pose_age_s').value)
        self.tag_id = int(node.get_parameter('mapping_tag_id').value)
        self.allowed_tag_ids = set(
            map(int, node.get_parameter('mapping_allowed_tag_ids').value))
        if -1 in self.allowed_tag_ids:
            self.allowed_tag_ids = set()
        family = str(node.get_parameter('mapping_tag_dictionary').value)
        dictionary_id = getattr(cv2.aruco, family, None)
        if dictionary_id is None:
            raise ValueError(f'建图视觉不支持标记字典 {family}')
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.detector_params = (cv2.aruco.DetectorParameters()
                                if hasattr(cv2.aruco, 'DetectorParameters') else
                                cv2.aruco.DetectorParameters_create())
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(dictionary, self.detector_params)
        else:
            self.detector = None
        self.dictionary = dictionary
        self._last_tag_log = float('-inf')
        node.get_logger().info(
            f'AprilTag字典={family}，期望ID={self.tag_id}，'
            f'允许ID={sorted(self.allowed_tag_ids) or "字典内全部"}')
        block = int(node.get_parameter('mapping_sgbm_block_size').value)
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=int(node.get_parameter('mapping_sgbm_num_disparities').value),
            blockSize=block, P1=8*block*block, P2=32*block*block,
            disp12MaxDiff=1, uniquenessRatio=8,
            speckleWindowSize=80, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def _pose_cb(self, msg):
        with self.lock:
            self.poses.append(msg)

    def _info_cb(self, side, msg):
        with self.lock:
            self.infos[side] = msg

    def _prepare(self, shape):
        height, width = shape
        if self.calibration is None:
            with self.lock:
                infos = dict(self.infos)
            if set(infos) != {'left', 'right'}:
                return False
            self.calibration = StereoCalibration.from_camera_info(
                'down_mapping', infos['left'], infos['right'],
                self.left_translation, self.camera_rotation,
                self.right_translation, self.camera_rotation)
            self.node.get_logger().info(
                f'建图视觉标定就绪：基线={self.calibration.baseline_m:.3f}m')
        if self.size == (width, height):
            return True
        # A fixed NPZ must match the actual V4L2 image size; silently scaling
        # its K/P matrices would make world positions look plausible but wrong.
        if self.sim_mode:
            with self.lock:
                info = self.infos.get('left')
            if info is None or (info.width, info.height) != (width, height):
                return False
        else:
            expected = (int(self.node.get_parameter('mapping_calibration_width').value),
                        int(self.node.get_parameter('mapping_calibration_height').value))
            if (width, height) != expected:
                raise ValueError(
                    f'真机标定尺寸{expected}与采集尺寸{(width, height)}不一致；'
                    '请使用当前相机模式重新标定并配置 mapping_calibration_file/width/height')
        self.maps = tuple(cv2.initUndistortRectifyMap(
            k, d, rect, proj[:, :3], (width, height), cv2.CV_32FC1)
            for k, d, rect, proj in (
                (self.calibration.camera_matrix_left, self.calibration.dist_left,
                 self.calibration.rectification_left, self.calibration.projection_left),
                (self.calibration.camera_matrix_right, self.calibration.dist_right,
                 self.calibration.rectification_right, self.calibration.projection_right)))
        self.size = (width, height)
        return True

    def _pose_for(self, stamp):
        with self.lock:
            poses = list(self.poses)
        if not poses:
            return None, None
        capture = _seconds(stamp)
        samples_by_stamp = {_seconds(pose.stamp): pose for pose in poses}
        samples = sorted(samples_by_stamp.items())
        times = [sample[0] for sample in samples]
        right_index = bisect_right(times, capture)
        if right_index == 0 or right_index == len(samples):
            pose_time, pose = min(samples, key=lambda item: abs(item[0] - capture))
            age = abs(pose_time - capture)
            return (pose, age) if age <= self.max_pose_age else (None, age)

        before_time, before = samples[right_index - 1]
        after_time, after = samples[right_index]
        if before_time == capture:
            return before, 0.0
        span = after_time - before_time
        age = max(capture - before_time, after_time - capture)
        if span <= 0.0 or age > self.max_pose_age:
            return None, age
        fraction = (capture - before_time) / span
        interpolated = PoseInfo()
        interpolated.stamp = stamp
        for field in ('robot_x', 'robot_y', 'robot_z', 'robot_roll',
                      'robot_pitch'):
            setattr(interpolated, field,
                    float(getattr(before, field)) + fraction *
                    (float(getattr(after, field)) - float(getattr(before, field))))
        yaw0, yaw1 = float(before.robot_yaw), float(after.robot_yaw)
        yaw_delta = (yaw1 - yaw0 + 180.0) % 360.0 - 180.0
        interpolated.robot_yaw = yaw0 + fraction * yaw_delta
        return interpolated, age

    def _depth_mode(self, polygon, depth):
        polygon = np.asarray(polygon, dtype=np.int32).reshape(-1, 2)
        if len(polygon) < 3:
            return None
        mask = np.zeros(depth.shape, np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        values = depth[mask != 0]
        values = values[np.isfinite(values) & (values >= self.min_depth)
                        & (values <= self.max_depth)]
        if len(values) < self.min_points:
            return None
        bins = np.arange(self.min_depth, self.max_depth + self.bin_m, self.bin_m)
        counts, edges = np.histogram(values, bins=bins)
        peak = int(np.argmax(counts))
        selected = values[(values >= edges[peak]) & (values < edges[peak+1])]
        if len(selected) < max(self.min_points, int(len(values)*self.peak_ratio)):
            return None
        # Use only pixels in the depth peak; the full mask centroid can be
        # biased strongly by a large background segment.
        peak_mask = (mask != 0) & np.isfinite(depth) & \
            (depth >= edges[peak]) & (depth < edges[peak+1])
        ys, xs = np.where(peak_mask)
        return float(np.median(selected)), (float(np.median(xs)), float(np.median(ys))), len(selected)

    def _cone_center(self, polygon, depth, pose):
        """Locate the cone axis above its base, not a visible surface patch."""
        polygon = np.asarray(polygon, dtype=np.int32).reshape(-1, 2)
        if len(polygon) < 3:
            return None
        mask = np.zeros(depth.shape, np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        moments = cv2.moments(mask)
        if moments['m00'] <= 0:
            return None
        pixel = (moments['m10']/moments['m00'],
                 moments['m01']/moments['m00'])
        # The old dominant mode is a useful fallback estimate of base/floor
        # height, but its pixel median is often on one side of the skirt.
        reference = self._depth_mode(polygon, depth)
        reference_z = (float(self._world(reference[1], reference[0], pose)[2])
                       if reference is not None else None)
        reference_count = reference[2] if reference is not None else 0

        orientation = _rpy_matrix(pose.robot_roll, pose.robot_pitch,
                                  pose.robot_yaw)
        transform = (orientation @ self.camera_rotation @
                     self.calibration.rectification_left.T)
        camera_z = (pose.robot_z +
                    float((orientation @ self.left_translation)[2]))
        projection = self.calibration.projection_left
        fx, fy = projection[0, 0], projection[1, 1]
        cx, cy = projection[0, 2], projection[1, 2]

        _, _, width, height = cv2.boundingRect(polygon)
        radius = max(8, min(30, int(min(width, height)*0.15)))
        outer = cv2.dilate(mask, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2*radius+1, 2*radius+1)))
        inner = cv2.dilate(mask, np.ones((7, 7), np.uint8))
        annulus = ((outer != 0) & (inner == 0) & np.isfinite(depth) &
                   (depth >= self.min_depth) & (depth <= self.max_depth))
        ys, xs = np.where(annulus)
        floor_z, count = reference_z, reference_count
        if len(xs) >= 80:
            xs, ys = xs[::4], ys[::4]
            distances = depth[ys, xs]
            ray_z = (transform[2, 0]*(xs-cx)/fx +
                     transform[2, 1]*(ys-cy)/fy + transform[2, 2])
            floor_samples = camera_z + distances*ray_z
            median = float(np.median(floor_samples))
            residuals = np.abs(floor_samples - median)
            mad = float(np.median(residuals))
            inliers = residuals < max(0.06, 3*mad)
            gap = median - reference_z if reference_z is not None else 0.0
            if (int(inliers.sum()) >= 60 and mad <= 0.08
                    and -0.15 <= gap <= 1.3*self.cone_height_m):
                floor_z, count = float(np.median(floor_samples[inliers])), \
                    int(inliers.sum())

        if floor_z is None:
            return None

        center_ray_z = (transform[2, 0]*(pixel[0]-cx)/fx +
                        transform[2, 1]*(pixel[1]-cy)/fy + transform[2, 2])
        if center_ray_z <= 0.2:
            return None
        distance = (floor_z-camera_z)/center_ray_z
        if not self.min_depth <= distance <= self.max_depth:
            return None
        world = self._world(pixel, distance, pose)
        world[2] = floor_z - self.cone_height_m/2
        return float(distance), pixel, count, world

    def _rectify_polygon(self, polygon):
        points = np.asarray(polygon, np.float32).reshape(-1, 1, 2)
        return cv2.undistortPoints(
            points, self.calibration.camera_matrix_left,
            self.calibration.dist_left,
            R=self.calibration.rectification_left,
            P=self.calibration.projection_left).reshape(-1, 2)

    def _world(self, pixel, distance, pose):
        p = self.calibration.projection_left
        ray = np.array([(pixel[0]-p[0, 2])*distance/p[0, 0],
                        (pixel[1]-p[1, 2])*distance/p[1, 1], distance])
        optical = self.calibration.rectification_left.T @ ray
        position = np.array([pose.robot_x, pose.robot_y, pose.robot_z])
        return position + _rpy_matrix(
            pose.robot_roll, pose.robot_pitch, pose.robot_yaw) @ (
                self.left_translation + self.camera_rotation @ optical)

    def debug_jpeg(self, name):
        with self.debug_lock:
            return self.debug_frames.get(name)

    def _update_debug(self, frame, left, disparity, depth, detections, stamp, pose):
        if time.monotonic() - self.debug_stamp < 0.75:
            return
        self.debug_stamp = time.monotonic()
        overlay = left.copy()
        for detection in detections.detections:
            if int(detection.class_id) not in (0, 1):
                continue
            polygon = np.column_stack((detection.mask_x, detection.mask_y))
            if len(polygon) < 3:
                continue
            polygon = self._rectify_polygon(polygon).astype(np.int32)
            color = ((0, 165, 255) if detection.class_id == 0 else
                     (255, 150, 0))
            mask = np.zeros(left.shape[:2], np.uint8)
            cv2.fillPoly(mask, [polygon], 1)
            overlay[mask != 0] = (0.55*overlay[mask != 0]
                                  + 0.45*np.asarray(color)).astype(np.uint8)
            cv2.polylines(overlay, [polygon], True, color, 2)
            sample = self._cone_center(polygon, depth, pose)
            if sample is not None:
                distance, pixel, _, _ = sample
                point = (int(round(pixel[0])), int(round(pixel[1])))
                cv2.drawMarker(overlay, point, (255, 255, 255),
                               cv2.MARKER_CROSS, 14, 2)
                cv2.putText(overlay, f'base {distance:.2f}m',
                            (point[0]+8, point[1]-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        def colorize(values, scale):
            finite = np.isfinite(values)
            image = np.zeros(values.shape, np.uint8)
            image[finite] = np.clip(values[finite]*scale, 0, 255).astype(np.uint8)
            return cv2.applyColorMap(image, cv2.COLORMAP_TURBO)
        valid = depth[np.isfinite(depth) & (depth >= self.min_depth)
                      & (depth <= self.max_depth)]
        counts, edges = np.histogram(
            valid, bins=np.arange(self.min_depth,
                                  self.max_depth + self.bin_m, self.bin_m))
        histogram = np.full((300, 640, 3), (13, 17, 23), np.uint8)
        if counts.size and counts.max():
            heights = (counts / counts.max() * 250).astype(np.int32)
            for x in range(640):
                index = min(int(x * len(heights) / 640), len(heights)-1)
                cv2.line(histogram, (x, 275), (x, 275-heights[index]),
                         (225, 208, 77), 1)
        cv2.putText(histogram, f'depth {self.min_depth:.1f}-{self.max_depth:.1f} m',
                    (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1)
        images = {'input': frame, 'overlay': overlay,
                  'disparity': colorize(disparity, 255/64),
                  'depth': colorize(depth, 255/self.max_depth),
                  'histogram': histogram}
        encoded = {}
        for name, image in images.items():
            if image.shape[1] > 960:
                image = cv2.resize(image, (960, int(image.shape[0]*960/image.shape[1])))
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 78])
            if ok:
                encoded[name] = (jpeg.tobytes(), stamp)
        with self.debug_lock:
            self.debug_frames = encoded

    def _detect_tags(self, raw, rectified):
        """Decode raw and rectified views without mixing their corner coordinates."""
        found, seen = [], set()
        rejected_count = 0
        for image, is_rectified in ((raw, False), (rectified, True)):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            for variant in (gray, cv2.createCLAHE(
                    clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)):
                corners, ids, rejected = (
                    self.detector.detectMarkers(variant) if self.detector is not None
                    else cv2.aruco.detectMarkers(
                        variant, self.dictionary, parameters=self.detector_params))
                rejected_count += len(rejected)
                if ids is None:
                    continue
                for polygon, tag_id in zip(corners, ids.flatten()):
                    tag_id = int(tag_id)
                    if tag_id not in seen:
                        found.append((tag_id, polygon.reshape(-1, 2), is_rectified))
                        seen.add(tag_id)
        return found, rejected_count

    def process(self, frame, stamp, left_detections, right_detections):
        """Called after YOLO on the exact same in-memory stitched BGR frame."""
        result = MappingObservationArray()
        result.header.stamp = stamp
        result.header.frame_id = 'mapping_odom'
        result.processed = False
        try:
            mid = frame.shape[1] // 2
            left, right = frame[:, :mid], frame[:, mid:mid*2]
            raw_left = left
            if not self._prepare(left.shape[:2]):
                result.reason = 'camera calibration unavailable'
                return
            pose, age = self._pose_for(stamp)
            if pose is None:
                result.reason = f'capture pose unavailable (age={age})'
                return
            left = cv2.remap(left, *self.maps[0], cv2.INTER_LINEAR)
            right = cv2.remap(right, *self.maps[1], cv2.INTER_LINEAR)
            disparity = self.sgbm.compute(
                cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
                cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)).astype(np.float32)/16.0
            depth = np.full(disparity.shape, np.nan, np.float32)
            good = disparity > 1.0
            depth[good] = (self.calibration.projection_left[0, 0]
                           * self.calibration.baseline_m / disparity[good])
            result.processed = True
            result.reason = 'ok'

            def append(kind, tag_id, class_id, confidence, polygon, rectified):
                if not rectified:
                    points = self._rectify_polygon(polygon)
                else:
                    points = polygon
                sample = (self._cone_center(points, depth, pose)
                          if kind == MappingObservation.CONE else
                          self._depth_mode(points, depth))
                if sample is None:
                    if kind == MappingObservation.TAG:
                        result.tag_depth_rejected += 1
                    else:
                        result.cone_depth_rejected += 1
                    return
                if kind == MappingObservation.CONE:
                    distance, pixel, count, world = sample
                else:
                    distance, pixel, count = sample
                    world = self._world(pixel, distance, pose)
                if not np.all(np.isfinite(world)):
                    if kind == MappingObservation.TAG:
                        result.tag_depth_rejected += 1
                    else:
                        result.cone_depth_rejected += 1
                    return
                observation = MappingObservation()
                observation.header = result.header
                observation.kind = kind
                observation.tag_id = tag_id
                observation.class_id = class_id
                observation.confidence = confidence
                observation.depth_m = distance
                observation.depth_samples = count
                observation.world_x, observation.world_y, observation.world_z = map(float, world)
                observation.pose_age_s = float(age)
                result.observations.append(observation)

            tags, rejected = self._detect_tags(raw_left, left)
            filtered_ids = []
            for tag_id, polygon, rectified in tags:
                result.tag_candidates += 1
                if ((not self.allowed_tag_ids or tag_id in self.allowed_tag_ids)
                        and (self.tag_id < 0 or tag_id == self.tag_id)):
                    append(MappingObservation.TAG, tag_id, -1, 1.0, polygon, rectified)
                else:
                    filtered_ids.append(tag_id)
            if time.monotonic() - self._last_tag_log >= 3.0:
                self._last_tag_log = time.monotonic()
                self.node.get_logger().info(
                    f'AprilTag诊断：解码ID={[item[0] for item in tags]}，'
                    f'字典={self.node.get_parameter("mapping_tag_dictionary").value}，'
                    f'左目原图={raw_left.shape[1]}x{raw_left.shape[0]}，'
                    f'被ID条件排除={filtered_ids}，本帧多预处理未解码候选数={rejected}，'
                    f'深度失败={result.tag_depth_rejected}；'
                    '未解码请检查字典、黑色编码边框及外围留白，镜像图像需单独复核')
            for detection in left_detections.detections:
                class_id = int(detection.class_id)
                if class_id in (0, 1):
                    result.cone_candidates += 1
                if class_id not in (0, 1):
                    continue
                if detection.confidence < self.min_confidence:
                    result.cone_confidence_rejected += 1
                    continue
                if len(detection.mask_x) < 3 or len(detection.mask_x) != len(detection.mask_y):
                    result.cone_mask_rejected += 1
                    continue
                polygon = np.column_stack((detection.mask_x, detection.mask_y))
                append(MappingObservation.CONE, -1, class_id,
                       float(detection.confidence), polygon, False)
        except (cv2.error, ValueError, TypeError) as error:
            result.processed = False
            result.reason = str(error)
            now = time.monotonic()
            if now - self.last_error_log > 3.0:
                self.node.get_logger().warning(f'建图视觉帧处理失败：{error}')
                self.last_error_log = now
        finally:
            self.pub.publish(result)
            if result.processed:
                try:
                    self._update_debug(
                        frame, left, disparity, depth, left_detections, stamp, pose)
                except (cv2.error, ValueError, TypeError) as error:
                    self.node.get_logger().warning(
                        f'建图可视化快照更新失败（观测已发布）：{error}')
