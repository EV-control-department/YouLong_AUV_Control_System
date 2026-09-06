"""LabelMe annotations -> YOLO detection (bbox-only) dataset.

The runtime detector only consumes ``result.boxes``.  Real deployment can
therefore use ordinary bounding-box annotations; segmentation polygons and
keypoints are not required.  This converter accepts LabelMe rectangles and,
for convenience, converts polygons/circles to their enclosing rectangles.

Example::

    python scripts/labelme2yolo_bbox.py \
        --labelme datas/down_dataset/labelme \
        --image-root datas/down_dataset/images \
        --classes datas/down_dataset/classes.txt \
        --output-root datas/down_bbox_dataset \
        --val-ratio 0.2

For a LabelMe directory containing full side-by-side stereo frames, add
``--split-side-by-side``.  The converter writes left/right half images and
clips/translates the large boxes into each eye's coordinate system.

The generated layout follows Ultralytics' standard detection convention::

    output-root/
      images/{train,val}/
      labels/{train,val}/
      data.yaml

Images are symlinked by default to avoid duplicating large recordings.  Use
``--copy-images`` when the output must be self-contained for deployment.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
from pathlib import Path


IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def load_classes(path: str | os.PathLike) -> tuple[dict[str, int], list[str]]:
    """Load one class name per line and return name->id plus ordered names."""
    names = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            name = line.strip()
            if name:
                names.append(name)
    return {name: index for index, name in enumerate(names)}, names


def _point_pair(value):
    try:
        x, y = value
        return float(x), float(y)
    except (TypeError, ValueError):
        return None


def shape_to_bbox(shape: dict) -> tuple[float, float, float, float] | None:
    """Return pixel ``(xmin, ymin, xmax, ymax)`` for a LabelMe shape."""
    shape_type = str(shape.get("shape_type", "")).strip().lower()
    points = shape.get("points") or []

    if shape_type == "rectangle" and len(points) >= 2:
        first = _point_pair(points[0])
        second = _point_pair(points[1])
        if first is None or second is None:
            return None
        return (
            min(first[0], second[0]), min(first[1], second[1]),
            max(first[0], second[0]), max(first[1], second[1]),
        )

    if shape_type in {"polygon", "linestrip"} and len(points) >= 2:
        values = [_point_pair(point) for point in points]
        if any(point is None for point in values):
            return None
        xs = [point[0] for point in values]
        ys = [point[1] for point in values]
        return min(xs), min(ys), max(xs), max(ys)

    # LabelMe circles store center and a point on the circumference.  They
    # are not required for bbox-only labeling, but converting them makes the
    # tool useful for datasets that already contain circular annotations.
    if shape_type == "circle" and len(points) >= 2:
        center = _point_pair(points[0])
        edge = _point_pair(points[1])
        if center is None or edge is None:
            return None
        radius_x = abs(edge[0] - center[0])
        radius_y = abs(edge[1] - center[1])
        radius = max(radius_x, radius_y)
        return (
            center[0] - radius, center[1] - radius,
            center[0] + radius, center[1] + radius,
        )

    return None


def _normalise_bbox(bbox, image_width: float, image_height: float):
    xmin, ymin, xmax, ymax = bbox
    xmin = max(0.0, min(float(image_width), xmin))
    ymin = max(0.0, min(float(image_height), ymin))
    xmax = max(0.0, min(float(image_width), xmax))
    ymax = max(0.0, min(float(image_height), ymax))
    width = xmax - xmin
    height = ymax - ymin
    if width <= 0.0 or height <= 0.0:
        return None
    return (
        ((xmin + xmax) * 0.5) / image_width,
        ((ymin + ymax) * 0.5) / image_height,
        width / image_width,
        height / image_height,
    )


def convert_one(data: dict, classes: dict[str, int], crop_x: float = 0.0,
                crop_width: float | None = None) -> tuple[str, set[str]]:
    """Convert one LabelMe JSON object to YOLO bbox lines."""
    source_width = data.get("imageWidth", data.get("image_width"))
    source_height = data.get("imageHeight", data.get("image_height"))
    try:
        source_width = float(source_width)
        source_height = float(source_height)
    except (TypeError, ValueError):
        return "", {"<missing image size>"}
    if source_width <= 0.0 or source_height <= 0.0:
        return "", {"<invalid image size>"}
    crop_x = max(0.0, float(crop_x))
    image_width = (source_width if crop_width is None
                   else min(float(crop_width), source_width - crop_x))
    if image_width <= 0.0:
        return "", {"<invalid crop size>"}

    lines = []
    skipped = set()
    for shape in data.get("shapes", []):
        label = str(shape.get("label", "")).strip()
        if label not in classes:
            skipped.add(label or "<empty label>")
            continue
        bbox = shape_to_bbox(shape)
        if bbox is None:
            skipped.add(f"{label}:unsupported_or_invalid_shape")
            continue
        xmin, ymin, xmax, ymax = bbox
        if crop_width is not None:
            # A target touching the stereo seam is represented by the visible
            # portion in each eye.  This is preferable to silently assigning
            # the full-frame coordinates to a half-width training image.
            xmin = max(xmin, crop_x)
            xmax = min(xmax, crop_x + image_width)
            xmin -= crop_x
            xmax -= crop_x
        normalised = _normalise_bbox(
            (xmin, ymin, xmax, ymax), image_width, source_height)
        if normalised is None:
            skipped.add(f"{label}:zero_area_bbox")
            continue
        cx, cy, width, height = normalised
        lines.append(
            f"{classes[label]} {cx:.6f} {cy:.6f} "
            f"{width:.6f} {height:.6f}"
        )
    return "\n".join(lines) + ("\n" if lines else ""), skipped


def _find_annotation(labelme_root: Path, image_path: Path) -> Path | None:
    candidate = labelme_root / f"{image_path.stem}.json"
    return candidate if candidate.is_file() else None


def _place_image(source: Path, destination: Path, copy_images: bool):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        destination.unlink()
    if copy_images:
        shutil.copy2(source, destination)
    else:
        destination.symlink_to(source.resolve())


def _write_split_images(source: Path, left: Path, right: Path):
    """Write the two eye images from one side-by-side source image."""
    try:
        import cv2
    except ImportError as error:
        raise RuntimeError(
            "--split-side-by-side requires OpenCV (python3-opencv)") from error
    image = cv2.imread(str(source), cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim < 2:
        raise ValueError(f"cannot read image for stereo split: {source}")
    height, width = image.shape[:2]
    if width < 2 or width % 2:
        raise ValueError(
            f"side-by-side image width must be positive and even: {source}")
    midpoint = width // 2
    left.parent.mkdir(parents=True, exist_ok=True)
    right.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(left), image[:, :midpoint]):
        raise IOError(f"cannot write left stereo image: {left}")
    if not cv2.imwrite(str(right), image[:, midpoint:]):
        raise IOError(f"cannot write right stereo image: {right}")
    return width, height, midpoint


def _write_data_yaml(output_root: Path, names: list[str], val_path: str):
    lines = [
        f"path: {output_root.resolve()}",
        "train: images/train",
        f"val: {val_path}",
        "",
        "names:",
    ]
    lines.extend(f"  {index}: {json.dumps(name, ensure_ascii=False)}"
                 for index, name in enumerate(names))
    (output_root / "data.yaml").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")


def build_dataset(labelme_root: str | os.PathLike,
                  image_root: str | os.PathLike,
                  classes_path: str | os.PathLike,
                  output_root: str | os.PathLike,
                  val_ratio: float = 0.2, seed: int = 0,
                  copy_images: bool = False,
                  split_side_by_side: bool = False):
    """Build a bbox-only dataset and return conversion statistics."""
    labelme_root = Path(labelme_root)
    image_root = Path(image_root)
    output_root = Path(output_root)
    classes, names = load_classes(classes_path)
    if not classes:
        raise ValueError("classes file is empty")
    if not image_root.is_dir():
        raise FileNotFoundError(f"image directory not found: {image_root}")
    if not labelme_root.is_dir():
        raise FileNotFoundError(f"LabelMe directory not found: {labelme_root}")

    images = sorted(
        path for path in image_root.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    if not images:
        raise ValueError(f"no images found in {image_root}")

    val_ratio = max(0.0, min(0.9, float(val_ratio)))
    shuffled = list(images)
    random.Random(int(seed)).shuffle(shuffled)
    val_count = int(round(len(shuffled) * val_ratio))
    if val_ratio > 0.0 and len(shuffled) > 1:
        val_count = max(1, min(len(shuffled) - 1, val_count))
    val_set = {path for path in shuffled[-val_count:]} if val_count else set()

    converted = 0
    negatives = 0
    shape_skips = set()
    output_image_count = 0
    output_train_count = 0
    output_val_count = 0
    for image_path in images:
        split = "val" if image_path in val_set else "train"
        annotation_path = _find_annotation(labelme_root, image_path)
        data = None
        if annotation_path is not None:
            data = json.loads(annotation_path.read_text(encoding="utf-8"))

        if not split_side_by_side:
            destination_image = output_root / "images" / split / image_path.name
            destination_label = (
                output_root / "labels" / split / f"{image_path.stem}.txt")
            _place_image(image_path, destination_image, copy_images)
            content = ""
            if data is not None:
                content, skipped = convert_one(data, classes)
                shape_skips.update(skipped)
            destination_label.parent.mkdir(parents=True, exist_ok=True)
            destination_label.write_text(content, encoding="utf-8")
            output_image_count += 1
            if split == "train":
                output_train_count += 1
            else:
                output_val_count += 1
            if content:
                converted += 1
            else:
                negatives += 1
            continue

        left_name = f"{image_path.stem}_left{image_path.suffix}"
        right_name = f"{image_path.stem}_right{image_path.suffix}"
        left_image = output_root / "images" / split / left_name
        right_image = output_root / "images" / split / right_name
        actual_width, actual_height, half_width = _write_split_images(
            image_path, left_image, right_image)
        for eye_name, crop_x in (("left", 0.0), ("right", half_width)):
            destination_label = (
                output_root / "labels" / split
                / f"{image_path.stem}_{eye_name}.txt")
            content = ""
            if data is not None:
                # LabelMe normally stores the true source dimensions.  If an
                # old JSON omitted them, provide the actual image dimensions.
                annotation = dict(data)
                annotation.setdefault("imageWidth", actual_width)
                annotation.setdefault("imageHeight", actual_height)
                content, skipped = convert_one(
                    annotation, classes, crop_x=crop_x,
                    crop_width=half_width)
                shape_skips.update(skipped)
            destination_label.parent.mkdir(parents=True, exist_ok=True)
            destination_label.write_text(content, encoding="utf-8")
            output_image_count += 1
            if split == "train":
                output_train_count += 1
            else:
                output_val_count += 1
            if content:
                converted += 1
            else:
                negatives += 1

    train_count = output_train_count
    val_count = output_val_count
    val_path = "images/val" if val_count else "images/train"
    _write_data_yaml(output_root, names, val_path)
    return {
        "images": output_image_count,
        "train": train_count,
        "val": val_count,
        "positive_images": converted,
        "negative_images": negatives,
        "skipped": sorted(shape_skips),
        "output_root": str(output_root),
    }


def main():
    parser = argparse.ArgumentParser(
        description="LabelMe -> YOLO detection bbox-only dataset")
    parser.add_argument("--labelme", default="datas/down_dataset/labelme")
    parser.add_argument("--image-root", default="datas/down_dataset/images")
    parser.add_argument("--classes", default="datas/down_dataset/classes.txt")
    parser.add_argument("--output-root", default="datas/down_bbox_dataset")
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--copy-images", action="store_true",
        help="copy images instead of creating symlinks")
    parser.add_argument(
        "--split-side-by-side", action="store_true",
        help=("split each full-width left|right image into two training "
              "images and transform/clamp bbox coordinates"))
    args = parser.parse_args()

    stats = build_dataset(
        args.labelme, args.image_root, args.classes, args.output_root,
        args.val_ratio, args.seed, args.copy_images,
        args.split_side_by_side)
    print(
        f"完成 bbox 数据集: images={stats['images']} "
        f"train={stats['train']} val={stats['val']} "
        f"positive={stats['positive_images']} "
        f"negative={stats['negative_images']} "
        f"output={stats['output_root']}"
    )
    if stats["skipped"]:
        print("跳过/异常标注:")
        for item in stats["skipped"]:
            print(f"  - {item}")


if __name__ == "__main__":
    main()
