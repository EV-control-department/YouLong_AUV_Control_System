"""下视相机 YOLO 分割模型迁移训练 (fine-tune)。

以预训练权重 WUURCnano0815.pt 为起点,在人工标注的下视数据集上继续训练。
训练前先运行 scripts/labelme2yolo.py 把 labelme JSON 转成 YOLO seg .txt。

用法:
    python scripts/train_down.py                          # 默认参数
    python scripts/train_down.py --imgsz 1280 --epochs 200 --batch 4
    python scripts/train_down.py --freeze 10              # 冻结前10层(backbone)

输出: runs/segment/train/ 下的 best.pt / last.pt。
"""

import argparse
import os

from ultralytics import YOLO

# 默认路径(相对仓库根目录)
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_WEIGHTS = os.path.join(ROOT, 'datas', 'WUURCnano0815.pt')
DEFAULT_DATA = os.path.join(ROOT, 'datas', 'down_dataset', 'data.yaml')


def main():
    ap = argparse.ArgumentParser(description='下视 YOLO 分割迁移训练')
    ap.add_argument('--weights', default=DEFAULT_WEIGHTS,
                    help='预训练权重 (起始点), 默认 WUURCnano0815.pt')
    ap.add_argument('--data', default=DEFAULT_DATA, help='数据集 data.yaml')
    ap.add_argument('--imgsz', type=int, default=640,
                    help='训练输入分辨率 (原图 2560x720, 建议 640~1280)')
    ap.add_argument('--epochs', type=int, default=100, help='训练轮数')
    ap.add_argument('--batch', type=int, default=8,
                    help='batch size (8GB 显存 seg 建议 4~8)')
    ap.add_argument('--lr', type=float, default=0.001, help='初始学习率')
    ap.add_argument('--freeze', type=int, default=0,
                    help='冻结前 N 层 (backbone=10; 0=全网络微调)')
    ap.add_argument('--project', default='runs/segment', help='输出根目录')
    ap.add_argument('--name', default='down_finetune', help='本次训练命名')
    args = ap.parse_args()

    print(f'→ 加载预训练权重: {args.weights}')
    model = YOLO(args.weights)

    print(f'→ 数据集: {args.data}')
    print(f'→ imgsz={args.imgsz}  epochs={args.epochs}  batch={args.batch}  '
          f'lr={args.lr}  freeze={args.freeze}')

    model.train(
        data=args.data,
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        lr0=args.lr,
        freeze=args.freeze,
        project=args.project,
        name=args.name,
        device=0,            # RTX 4060
        workers=4,
        seed=0,
        exist_ok=True,
        cache=False,
    )


if __name__ == '__main__':
    main()
