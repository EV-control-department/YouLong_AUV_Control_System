"""用预训练模型对下视帧做自动标注(伪标注)。

对每张图像跑 YOLO 分割推理,把高置信度掩膜转成 YOLO seg 多边形,
输出到 labels/ 供后续 fine-tune 使用。

用法:
    python scripts/auto_label.py \
        --images datas/down_dataset/images \
        --labels datas/down_dataset/labels \
        --weights datas/WUURCnano0815.pt \
        --conf 0.5 --imgsz 1280

说明:
- masks.xy 返回原始像素坐标下的多边形,做 approxPolyDP 简化后归一化。
- 面积过小(相对图像比例 < --min_area_ratio)或置信度 < --conf 的掩膜丢弃。
- 无掩膜/无检测的图像不生成 .txt(和手标"没标"一致)。
"""

import argparse
import os

import cv2
import numpy as np

from ultralytics import YOLO


def mask_to_yolo_line(class_id: int, poly_px, img_w: int, img_h: int,
                      min_area_ratio: float):
    """掩膜多边形(原始像素) → YOLO seg 行;过小返回 None。"""
    poly = np.asarray(poly_px, dtype=np.float32).reshape(-1, 2)
    if len(poly) < 3:
        return None

    # 面积过滤
    area = cv2.contourArea(poly.astype(np.float32))
    if area < min_area_ratio * (img_w * img_h):
        return None

    # 简化折线,减少标注点
    cnt = poly.reshape((-1, 1, 2)).astype(np.int32)
    cnt = cv2.approxPolyDP(cnt, 2.0, True)
    cnt = cnt.reshape(-1, 2)
    if len(cnt) < 3:
        return None

    parts = []
    for x, y in cnt:
        nx = max(0.0, min(1.0, float(x) / img_w))
        ny = max(0.0, min(1.0, float(y) / img_h))
        parts.append(f'{nx:.6f} {ny:.6f}')
    return f'{class_id} ' + ' '.join(parts)


def main():
    ap = argparse.ArgumentParser(description='YOLO 自动标注')
    ap.add_argument('--images', default='datas/down_dataset/images')
    ap.add_argument('--labels', default='datas/down_dataset/labels')
    ap.add_argument('--weights', default='datas/WUURCnano0815.pt')
    ap.add_argument('--conf', type=float, default=0.5, help='置信度阈值')
    ap.add_argument('--imgsz', type=int, default=1280, help='推理分辨率')
    ap.add_argument('--min_area_ratio', type=float, default=0.0005,
                    help='掩膜最小相对面积')
    args = ap.parse_args()

    os.makedirs(args.labels, exist_ok=True)
    files = sorted(f for f in os.listdir(args.images)
                   if f.lower().endswith(('.jpg', '.jpeg', '.png')))
    print(f'图像: {len(files)} 张 | 模型: {args.weights} | conf={args.conf} '
          f'imgsz={args.imgsz}')

    model = YOLO(args.weights)
    n_labels = 0
    n_empty = 0
    class_count = {}

    for i, fname in enumerate(files):
        img_path = os.path.join(args.images, fname)
        res = model.predict(img_path, conf=args.conf, imgsz=args.imgsz,
                            verbose=False)[0]

        h, w = res.orig_shape
        lines = []
        masks = res.masks
        if masks is not None and res.boxes is not None:
            for mi, poly in enumerate(masks.xy):
                cid = int(res.boxes.cls[mi])
                line = mask_to_yolo_line(cid, poly, w, h,
                                         args.min_area_ratio)
                if line:
                    lines.append(line)
                    class_count[cid] = class_count.get(cid, 0) + 1

        if lines:
            stem = os.path.splitext(fname)[0]
            with open(os.path.join(args.labels, stem + '.txt'),
                      'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
            n_labels += 1
        else:
            n_empty += 1

        if (i + 1) % 100 == 0:
            print(f'  已处理 {i + 1}/{len(files)} (标注 {n_labels}, 空 {n_empty})')

    print(f'完成: 生成标签 {n_labels} 张, 空(无检测) {n_empty} 张')
    print('按类别统计:')
    for cid in sorted(class_count):
        print(f'  {model.names[cid]:<15} (id={cid}): {class_count[cid]}')


if __name__ == '__main__':
    main()
