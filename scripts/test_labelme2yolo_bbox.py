"""Tests for the bbox-only LabelMe conversion tool."""

import json

import cv2
import numpy as np

from labelme2yolo_bbox import build_dataset, convert_one


def test_convert_rectangle_to_yolo_bbox():
    content, skipped = convert_one(
        {
            "imageWidth": 100,
            "imageHeight": 80,
            "shapes": [{
                "label": "ball",
                "shape_type": "rectangle",
                "points": [[10, 20], [50, 60]],
            }],
        },
        {"ball": 3},
    )

    assert skipped == set()
    assert content == "3 0.300000 0.500000 0.400000 0.500000\n"


def test_build_bbox_dataset_keeps_negative_images_and_split(tmp_path):
    image_root = tmp_path / "images"
    labelme_root = tmp_path / "labelme"
    output_root = tmp_path / "bbox_dataset"
    image_root.mkdir()
    labelme_root.mkdir()
    (tmp_path / "classes.txt").write_text("ball\n", encoding="utf-8")

    for name in ("a.jpg", "b.jpg", "c.jpg", "d.jpg"):
        (image_root / name).write_bytes(b"test")
    (labelme_root / "a.json").write_text(json.dumps({
        "imageWidth": 100,
        "imageHeight": 100,
        "shapes": [{
            "label": "ball",
            "shape_type": "rectangle",
            "points": [[10, 10], [30, 30]],
        }],
    }), encoding="utf-8")

    stats = build_dataset(
        labelme_root,
        image_root,
        tmp_path / "classes.txt",
        output_root,
        val_ratio=0.25,
        seed=0,
        copy_images=True,
    )

    assert stats["images"] == 4
    assert stats["train"] == 3
    assert stats["val"] == 1
    assert stats["positive_images"] == 1
    assert stats["negative_images"] == 3
    assert (output_root / "data.yaml").is_file()
    labels = list((output_root / "labels").rglob("*.txt"))
    assert len(labels) == 4
    assert any(path.read_text(encoding="utf-8") == "" for path in labels)
    positive = [path for path in labels
                if path.read_text(encoding="utf-8").strip()]
    assert len(positive) == 1
    assert len(positive[0].read_text(encoding="utf-8").split()) == 5


def test_build_split_stereo_dataset_translates_boxes_to_each_eye(tmp_path):
    image_root = tmp_path / "images"
    labelme_root = tmp_path / "labelme"
    output_root = tmp_path / "bbox_dataset"
    image_root.mkdir()
    labelme_root.mkdir()
    (tmp_path / "classes.txt").write_text("ball\n", encoding="utf-8")
    cv2.imwrite(
        str(image_root / "stereo.jpg"),
        np.zeros((40, 100, 3), dtype=np.uint8),
    )
    (labelme_root / "stereo.json").write_text(json.dumps({
        "imageWidth": 100,
        "imageHeight": 40,
        "shapes": [
            {"label": "ball", "shape_type": "rectangle",
             "points": [[10, 10], [30, 30]]},
            {"label": "ball", "shape_type": "rectangle",
             "points": [[60, 10], [90, 30]]},
        ],
    }), encoding="utf-8")

    stats = build_dataset(
        labelme_root, image_root, tmp_path / "classes.txt", output_root,
        val_ratio=0.0, copy_images=True, split_side_by_side=True,
    )

    assert stats["images"] == 2
    left = (output_root / "labels" / "train" / "stereo_left.txt").read_text()
    right = (output_root / "labels" / "train" / "stereo_right.txt").read_text()
    assert left == "0 0.400000 0.500000 0.400000 0.500000\n"
    assert right == "0 0.500000 0.500000 0.600000 0.500000\n"
