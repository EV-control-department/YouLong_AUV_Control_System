"""YOLO-seg + calibrated SGBM diagnostics for recorded down-stereo pairs.

The legacy XYZ is a surface sample in the left optical frame. The center
comparison uses a zero vehicle pose and is not a world map; recorded vehicle
poses are required to combine different frames.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from ultralytics import YOLO


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'workspace_auv/src/uv_camera'))
from uv_camera.down_calibration import load_real_down_json  # noqa: E402
from uv_camera.mapping_vision import MappingVision  # noqa: E402


def calibration(path):
    width, height, k1, d1, k2, d2, rotation, translation = load_real_down_json(path)
    r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
        k1, d1, k2, d2, (width, height), rotation, translation,
        flags=cv2.CALIB_ZERO_DISPARITY)
    maps = [cv2.initUndistortRectifyMap(k, d, r, p[:, :3],
                                       (width, height), cv2.CV_32FC1)
            for k, d, r, p in ((k1, d1, r1, p1), (k2, d2, r2, p2))]
    return (width, height, k1, d1, r1, p1,
            abs(float(p2[0, 3] / p2[0, 0])), maps)


def analyze(pair_index, dataset, model, params, sgbm, device):
    width, height, k1, d1, r1, p1, baseline, maps = params
    filename = f'{pair_index:06d}.png'
    originals = [cv2.imread(str(dataset / eye / filename))
                 for eye in ('left', 'right')]
    if any(frame is None or frame.shape[:2] != (height, width)
           for frame in originals):
        raise ValueError(f'{filename}: 缺少双目对或每目不是 {width}x{height}')
    rectified = [cv2.remap(frame, *mapping, cv2.INTER_LINEAR)
                 for frame, mapping in zip(originals, maps)]
    disparity = sgbm.compute(*[cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                               for frame in rectified]).astype(np.float32) / 16
    depth = np.full(disparity.shape, np.nan, np.float32)
    valid = disparity > 1.0
    depth[valid] = p1[0, 0] * baseline / disparity[valid]

    result = model.predict(originals[0], conf=0.35, verbose=False,
                           device=device)[0]
    if result.masks is None:
        raise ValueError(f'{filename}: YOLO 没有输出锥桶掩膜')
    candidates = [(float(score), index) for index, (class_id, score) in
                  enumerate(zip(result.boxes.cls, result.boxes.conf))
                  if int(class_id) in (0, 1)]
    if not candidates:
        raise ValueError(f'{filename}: YOLO 未识别出锥桶')
    confidence, index = max(candidates)
    class_id = int(result.boxes.cls[index])
    raw_polygon = np.asarray(result.masks.xy[index], np.float32).reshape(-1, 2)
    rectified_polygon = cv2.undistortPoints(
        raw_polygon.reshape(-1, 1, 2), k1, d1, R=r1, P=p1).reshape(-1, 2)
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillPoly(mask, [rectified_polygon.astype(np.int32)], 1)
    in_mask = (mask != 0) & np.isfinite(depth) & (depth >= 0.2) & (depth <= 8.0)
    values = depth[in_mask]
    if len(values) < 20:
        raise ValueError(f'{filename}: 掩膜内有效深度不足（{len(values)} 点）')
    counts, edges = np.histogram(values, bins=np.arange(0.2, 8.02, 0.02))
    peak = int(np.argmax(counts))
    peak_mask = in_mask & (depth >= edges[peak]) & (depth < edges[peak + 1])
    ys, xs = np.where(peak_mask)
    if len(xs) < max(20, int(0.12 * len(values))):
        raise ValueError(f'{filename}: 深度峰只有 {len(xs)}/{len(values)} 点')
    z = float(np.median(depth[peak_mask]))
    u, v = float(np.median(xs)), float(np.median(ys))
    xyz_rect = np.array([(u - p1[0, 2]) * z / p1[0, 0],
                         (v - p1[1, 2]) * z / p1[1, 1], z])
    xyz = r1.T @ xyz_rect
    selector = MappingVision.__new__(MappingVision)
    selector.min_depth, selector.max_depth = 0.2, 8.0
    selector.min_points, selector.bin_m, selector.peak_ratio = 20, 0.02, 0.12
    selector.cone_height_m = 0.5
    selector.calibration = SimpleNamespace(
        projection_left=p1, rectification_left=r1)
    selector.left_translation = np.array([-0.130, -0.0305, 0.0645])
    selector.camera_rotation = np.array([[0., -1., 0.],
                                         [1., 0., 0.],
                                         [0., 0., 1.]])
    pose = SimpleNamespace(robot_x=0., robot_y=0., robot_z=0.,
                           robot_roll=0., robot_pitch=0., robot_yaw=0.)
    center = selector._cone_center(rectified_polygon, depth, pose)
    if center is None:
        raise ValueError(f'{filename}: 无法确定锥桶中心')
    base_depth, center_pixel, center_count, center_world = center
    old_world = selector._world((u, v), z, pose)
    return dict(index=pair_index, filename=filename, class_id=class_id,
                class_name=model.names[class_id], confidence=confidence,
                left=originals[0], rectified_left=rectified[0],
                disparity=disparity, depth=depth,
                raw_polygon=raw_polygon, rectified_polygon=rectified_polygon,
                peak_mask=peak_mask, peak_pixel=(u, v), xyz=xyz,
                center_pixel=center_pixel, center_world=center_world,
                center_count=center_count, base_depth=base_depth,
                old_world=old_world,
                valid_count=len(values), peak_count=len(xs),
                histogram=(counts, edges), peak_bin=peak)


def save_selector_comparison(results, output):
    fig, axes = plt.subplots(len(results), 4, figsize=(20, 4.1*len(results)),
                             constrained_layout=True)
    if len(results) == 1:
        axes = axes[np.newaxis, :]
    for row, item in enumerate(results):
        rectified, depth_ax, disparity_ax, histogram = axes[row]
        contour = np.vstack((item['rectified_polygon'],
                             item['rectified_polygon'][0]))
        old = item['peak_pixel']
        new = item['center_pixel']
        for ax, image in ((rectified, cv2.cvtColor(item['rectified_left'], cv2.COLOR_BGR2RGB)),
                          (depth_ax, np.ma.masked_invalid(item['depth'])),
                          (disparity_ax, np.ma.masked_where(item['disparity'] <= 1,
                                                            item['disparity']))):
            if ax is rectified:
                ax.imshow(image)
            elif ax is depth_ax:
                ax.imshow(image, cmap='turbo', vmin=0.2, vmax=1.2)
            else:
                ax.imshow(image, cmap='magma', vmin=0, vmax=100)
            ax.plot(contour[:, 0], contour[:, 1], color='white', lw=1.5)
            ax.scatter(*old, marker='x', color='#ff3e4e', s=110, linewidths=2.5)
            ax.scatter(*new, marker='+', color='#35f5a6', s=150, linewidths=2.5)
            ax.set_xlim(0, item['depth'].shape[1])
            ax.set_ylim(item['depth'].shape[0], 0)
            ax.axis('off')
        rectified.set_title(f"#{item['index']:06d} {item['class_name']} | red skirt, green center")
        depth_ax.set_title(f"Base depth: {item['xyz'][2]:.3f} -> {item['base_depth']:.3f} m")
        disparity_ax.set_title('Rectified SGBM disparity / px')

        counts, edges = item['histogram']
        centers = (edges[:-1] + edges[1:]) / 2
        histogram.bar(centers, counts, width=0.019, color='#5b819e')
        histogram.axvline(item['xyz'][2], color='#ff3e4e', lw=2,
                          label=f"old mode {item['xyz'][2]:.3f} m")
        histogram.axvline(item['base_depth'], color='#0ca56b', lw=2,
                          label=f"local floor {item['base_depth']:.3f} m")
        histogram.set_xlim(0.2, 1.2)
        histogram.set_xlabel('Depth within YOLO mask / m')
        histogram.set_ylabel('pixels / 0.02 m')
        histogram.legend(fontsize=9)
        histogram.grid(alpha=0.2)
    fig.suptitle('Cone localization: skirt sample vs projected geometric center',
                 fontsize=16)
    fig.savefig(output, dpi=140)
    plt.close(fig)


def save_camera_comparison(results, output):
    fig = plt.figure(figsize=(11, 8), facecolor='white')
    ax = fig.add_subplot(111, projection='3d')
    for item in results:
        old = item['old_world']
        new = item['center_world']
        ax.plot([0, old[0]], [0, old[1]], [0, old[2]],
                color='#ff3e4e', alpha=0.5)
        ax.plot([0, new[0]], [0, new[1]], [0, new[2]],
                color='#0ca56b', alpha=0.5)
        ax.scatter(*old, color='#ff3e4e', s=70)
        ax.scatter(*new, color='#0ca56b', s=70)
        ax.plot([old[0], new[0]], [old[1], new[1]], [old[2], new[2]],
                color='#596675', linestyle=':', lw=1)
        ax.text(*new, f"#{item['index']:06d}", fontsize=9)
    ax.scatter([], [], [], color='#ff3e4e', label='old: global mode')
    ax.scatter([], [], [], color='#0ca56b', label='new: cone axis at mid-height')
    ax.legend(loc='upper left')
    ax.set(xlabel='X / m (vehicle)', ylabel='Y / m (vehicle)',
           zlabel='Z / m (down)')
    ax.set_title('Per-frame points with zero vehicle pose (not a world map)')
    ax.view_init(elev=25, azim=-65)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def save_short_sequences(groups, output):
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    for ax, group in zip(axes.flat, groups):
        frames = [item['index'] for item in group]
        old = [float(item['xyz'][2]) for item in group]
        new = [float(item['base_depth']) for item in group]
        ax.plot(frames, old, 'o-', color='#ff3e4e', label='old: skirt mode')
        ax.plot(frames, new, 'o-', color='#0ca56b', label='new: local floor')
        ax.set_title(f"Frames {frames[0]}-{frames[-1]} | "
                     f"old SD={np.std(old):.3f} m, new SD={np.std(new):.3f} m")
        ax.set_xlabel('Recorded frame')
        ax.set_ylabel('Optical depth / m')
        ax.set_ylim(0.2, 0.95)
        ax.grid(alpha=0.25)
        ax.legend(loc='best')
    fig.suptitle('Short sequences: base-depth estimates, not world-position error',
                 fontsize=15)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def save_montage(results, output):
    colors = {0: '#ffad36', 1: '#3ca7ff'}
    fig, axes = plt.subplots(len(results), 4, figsize=(20, 4.25 * len(results)),
                             constrained_layout=True)
    if len(results) == 1:
        axes = axes[np.newaxis, :]
    fig.patch.set_facecolor('#f5f7fa')
    for row, item in enumerate(results):
        raw, disparity, depth, histogram = axes[row]
        color = colors[item['class_id']]
        raw.imshow(cv2.cvtColor(item['left'], cv2.COLOR_BGR2RGB))
        polygon = item['raw_polygon']
        raw.fill(polygon[:, 0], polygon[:, 1], color=color, alpha=0.3)
        raw.plot(*np.vstack((polygon, polygon[0])).T, color=color, lw=2)
        raw.set_title(f"#{item['index']:06d}  {item['class_name']}  YOLO {item['confidence']:.2f}")
        raw.axis('off')

        disparity.imshow(np.ma.masked_where(item['disparity'] <= 1, item['disparity']),
                         cmap='magma', vmin=0, vmax=96)
        disparity.plot(item['peak_pixel'][0], item['peak_pixel'][1],
                       marker='+', color='cyan', ms=14, mew=2)
        disparity.set_title('Rectified SGBM disparity (0–96 px)')
        disparity.axis('off')

        depth.imshow(np.ma.masked_invalid(item['depth']), cmap='turbo',
                     vmin=0.3, vmax=1.6)
        contour = item['rectified_polygon']
        depth.plot(*np.vstack((contour, contour[0])).T, color='white', lw=1.5)
        depth.plot(item['peak_pixel'][0], item['peak_pixel'][1],
                   marker='+', color='black', ms=14, mew=2)
        depth.set_title(f"Depth (0.3–1.6 m); peak Z={item['xyz'][2]:.2f} m")
        depth.axis('off')

        counts, edges = item['histogram']
        centers = (edges[:-1] + edges[1:]) / 2
        histogram.bar(centers, counts, width=0.019, color='#5586b5')
        j = item['peak_bin']
        histogram.axvspan(edges[j], edges[j + 1], color='#ffad36', alpha=.8,
                          label=f"peak {item['xyz'][2]:.2f} m")
        histogram.set_xlim(0.2, 2.0)
        histogram.set_xlabel('Depth in YOLO mask / m')
        histogram.set_ylabel('pixels / 0.02 m')
        histogram.set_title(f"Mode: {item['peak_count']}/{item['valid_count']} pixels")
        histogram.legend(loc='upper right', fontsize=9)
        histogram.grid(alpha=.2)
    fig.suptitle('Real down-stereo: raw YOLO mask → rectification + SGBM → mask depth mode',
                 fontsize=17)
    fig.savefig(output, dpi=145, facecolor=fig.get_facecolor())
    plt.close(fig)


def save_camera_points(results, output):
    fig = plt.figure(figsize=(14, 8), facecolor='#f6f8fb')
    ax = fig.add_axes([.04, .09, .59, .80], projection='3d')
    colors = {0: '#eb891c', 1: '#2389d6'}
    for item in results:
        x, y, z = map(float, item['xyz'])
        color = colors[item['class_id']]
        ax.plot([0, x], [0, y], [0, z], color=color, alpha=.5, lw=1.7)
        ax.scatter([x], [y], [z], color=color, s=95, depthshade=False)
    ax.scatter([0], [0], [0], marker='^', color='#222222', s=140)
    ax.text(0, 0, -.06, 'left camera', fontsize=10)
    ax.set(xlim=(-.28, .28), ylim=(-.28, .28), zlim=(0, 1.15),
           xlabel='X: image right / m', ylabel='Y: image down / m',
           zlabel='Z: optical depth / m')
    ax.view_init(elev=22, azim=-62)
    fig.suptitle('Representative cone surface points in left-camera coordinates',
                 fontsize=17, y=.97)
    fig.text(.66, .82, 'Frame / class       X       Y       Z (m)',
             fontsize=10, family='monospace', weight='bold')
    for row, item in enumerate(results):
        x, y, z = map(float, item['xyz'])
        fig.text(.66, .75 - row * .115,
                 f"#{item['index']:06d}  {item['class_name']:<12} "
                 f'{x:+.3f}  {y:+.3f}  {z:.3f}',
                 fontsize=11, family='monospace', color=colors[item['class_id']])
    fig.text(.66, .21, 'X: image right\nY: image down\nZ: optical depth',
             fontsize=11, linespacing=1.5, color='#32445a')
    fig.text(.5, .035,
             'Each point belongs to a different moving-camera frame. This is NOT a world map.',
             ha='center', fontsize=11, color='#a32626')
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path,
                        default=PROJECT.parent / '20261002_174456_755376')
    parser.add_argument('--calibration', type=Path,
                        default=PROJECT / 'workspace_auv/docs/stereo_parameters.json')
    parser.add_argument('--model', type=Path,
                        default=PROJECT / 'workspace_auv/src/uv_camera/resource/last.pt')
    parser.add_argument('--frames', nargs='+', type=int,
                        default=[60, 180, 400, 680])
    parser.add_argument('--output', type=Path,
                        default=PROJECT / 'visualization/reports/20261002_174456_755376')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    params = calibration(args.calibration)
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=128, blockSize=5,
        P1=8 * 25, P2=32 * 25, disp12MaxDiff=1,
        uniquenessRatio=8, speckleWindowSize=80, speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    model = YOLO(str(args.model))
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    results = [analyze(i, args.dataset, model, params, sgbm, device)
               for i in args.frames]
    save_montage(results, args.output / 'yolo_sgbm_diagnostics.png')
    save_camera_points(results, args.output / 'camera_frame_points_3d.png')
    save_selector_comparison(results, args.output / 'cone_center_comparison.png')
    save_camera_comparison(results, args.output / 'cone_center_comparison_3d.png')
    groups = []
    cached = {item['index']: item for item in results}
    for center in (60, 180, 400, 680):
        group = []
        for index in range(center-2, center+3):
            if index not in cached:
                cached[index] = analyze(index, args.dataset, model, params,
                                        sgbm, device)
            group.append(cached[index])
        groups.append(group)
    save_short_sequences(groups, args.output / 'cone_center_depth_sequences.png')
    with (args.output / 'cone_center_comparison.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['frame', 'class', 'old_x_m', 'old_y_m', 'old_z_m',
                         'center_x_m', 'center_y_m', 'center_z_m',
                         'base_depth_m', 'depth_support_pixels'])
        for item in results:
            x, y, z = map(float, item['center_world'])
            old_x, old_y, old_z = map(float, item['old_world'])
            writer.writerow([item['filename'], item['class_name'],
                             f'{old_x:.4f}', f'{old_y:.4f}', f'{old_z:.4f}',
                             f'{x:.4f}', f'{y:.4f}', f'{z:.4f}',
                             f"{item['base_depth']:.4f}", item['center_count']])
    with (args.output / 'camera_frame_points.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['frame', 'class', 'confidence', 'x_m', 'y_m', 'z_m',
                         'valid_depth_pixels', 'peak_pixels'])
        for item in results:
            writer.writerow([item['filename'], item['class_name'],
                             f"{item['confidence']:.4f}",
                             *(f'{float(v):.4f}' for v in item['xyz']),
                             item['valid_count'], item['peak_count']])
    print(f'Generated {len(results)} frame reports in {args.output}')


if __name__ == '__main__':
    main()
