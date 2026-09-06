"""Train the bbox-only YOLO detector used by ``uv_camera``.

This is deliberately a detection task.  The runtime consumes bounding boxes
and centers, so real deployment does not require segmentation masks or
keypoint labels.

Example::

    python scripts/labelme2yolo_bbox.py
    python scripts/train_bbox.py --weights yolov8n.pt --imgsz 640 \
        --epochs 100 --batch 8
"""

from __future__ import annotations

import argparse
import os

from ultralytics import YOLO


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_DATA = os.path.join(ROOT, "datas", "down_bbox_dataset", "data.yaml")


def main():
    parser = argparse.ArgumentParser(
        description="Train the bbox-only YOLO detector")
    parser.add_argument(
        "--weights", default="yolov8n.pt",
        help="detection checkpoint or model name; must be a detect model")
    parser.add_argument("--data", default=DEFAULT_DATA)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--lr", type=float, default=0.001)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--device", default="0")
    parser.add_argument("--freeze", type=int, default=0)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--project", default="runs/detect")
    parser.add_argument("--name", default="bbox_finetune")
    args = parser.parse_args()

    print(f"→ 加载检测模型: {args.weights}")
    model = YOLO(args.weights)
    if getattr(model, "task", "detect") != "detect":
        raise SystemExit(
            f"{args.weights} 的任务类型是 {getattr(model, 'task', None)!r}，"
            "请提供 detection checkpoint，而不是 segmentation checkpoint")

    print(f"→ bbox 数据集: {args.data}")
    print(
        f"→ imgsz={args.imgsz} epochs={args.epochs} batch={args.batch} "
        f"lr={args.lr} device={args.device} workers={args.workers}"
    )
    model.train(
        task="detect",
        data=args.data,
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        lr0=args.lr,
        freeze=args.freeze,
        patience=args.patience,
        project=args.project,
        name=args.name,
        device=args.device,
        workers=max(0, args.workers),
        seed=0,
        exist_ok=True,
        cache=False,
    )


if __name__ == "__main__":
    main()
