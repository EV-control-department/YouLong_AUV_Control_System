"""labelme 标注 → YOLO 分割格式 (.txt) 转换脚本。

用法:
    python scripts/labelme2yolo.py \
        --labelme datas/down_dataset/labelme \
        --labels datas/down_dataset/labels \
        --classes datas/down_dataset/classes.txt

- 读取 datas/down_dataset/labelme/ 下每个 labelme JSON (多边形标注)。
- 只转换 shape_type == "polygon" 的形状;line/point 等其它类型跳过。
- 标签名必须在 classes.txt 里,否则跳过并警告。
- 输出 YOLO seg .txt 到 labels/,文件名与 JSON 同名 (不含扩展)。
"""

import argparse
import json
import os
import sys


def load_classes(path: str) -> dict:
    """读 classes.txt(每行一个类名) → {类名: id}。"""
    classes = {}
    with open(path, 'r', encoding='utf-8') as f:
        for i, line in enumerate(f):
            name = line.strip()
            if name:
                classes[name] = i
    return classes


def shape_to_yolo(points, image_w, image_h) -> str:
    """多边形点(像素) → YOLO seg 行的归一化坐标段。"""
    parts = []
    for x, y in points:
        nx = max(0.0, min(1.0, float(x) / image_w))
        ny = max(0.0, min(1.0, float(y) / image_h))
        parts.append(f'{nx:.6f} {ny:.6f}')
    return ' '.join(parts)


def convert_one(json_path: str, classes: dict, image_root: str) -> str:
    """转换单个 labelme JSON → YOLO seg .txt 内容。返回内容或 None。"""
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    image_w = data.get('imageWidth') or data.get('image_width')
    image_h = data.get('imageHeight') or data.get('image_height')
    if not image_w or not image_h:
        print(f'  [skip] 缺 imageWidth/Height: {os.path.basename(json_path)}')
        return None

    lines = []
    skipped_labels = set()
    for shape in data.get('shapes', []):
        label = shape.get('label')
        if shape.get('shape_type') != 'polygon':
            continue
        if label not in classes:
            skipped_labels.add(label)
            continue
        points = shape.get('points', [])
        if len(points) < 3:
            continue
        cid = classes[label]
        lines.append(f'{cid} {shape_to_yolo(points, image_w, image_h)}')

    if skipped_labels:
        print(f'  [warn] 未在 classes.txt 的标签被跳过: {sorted(skipped_labels)} '
              f'({os.path.basename(json_path)})')

    if not lines:
        return None
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser(description='labelme → YOLO seg 转换')
    ap.add_argument('--labelme', default='datas/down_dataset/labelme')
    ap.add_argument('--labels', default='datas/down_dataset/labels')
    ap.add_argument('--classes', default='datas/down_dataset/classes.txt')
    ap.add_argument('--image-root', default='datas/down_dataset/images',
                    help='用于核对图像文件是否存在(仅校验,可选)')
    args = ap.parse_args()

    classes = load_classes(args.classes)
    if not classes:
        print(f'classes.txt 为空或不存在: {args.classes}', file=sys.stderr)
        sys.exit(1)
    print(f'类表 ({len(classes)}): {classes}')

    os.makedirs(args.labels, exist_ok=True)
    json_files = sorted(
        f for f in os.listdir(args.labelme) if f.endswith('.json'))
    if not json_files:
        print(f'labelme 目录没有 JSON: {args.labelme}')
        sys.exit(0)

    converted = 0
    for jf in json_files:
        json_path = os.path.join(args.labelme, jf)
        content = convert_one(json_path, classes, args.image_root)
        if content is None:
            print(f'  [skip] 无有效多边形: {jf}')
            continue
        out_name = os.path.splitext(jf)[0] + '.txt'
        with open(os.path.join(args.labels, out_name), 'w', encoding='utf-8') as f:
            f.write(content)
        converted += 1

    print(f'完成: 转换 {converted}/{len(json_files)} 张到 {args.labels}')


if __name__ == '__main__':
    main()
