# 国水 2026「水中巡游」8029 数据集 YOLO 类别 ID 表

来源：`/home/doc049/dev/tmp/Database/8029/id.md`。本表以 `id.md` 中的顺序为准，YOLO ID 从 0 开始；仅适用于 8029 数据集。

> 注意：本表不能与当前工程 `datas/down_dataset/classes.txt` 的 0–8 类别混用。训练 8029 数据集时，类别名称和 ID 必须保持如下对应关系。

## 核心检测类别

| YOLO ID | `class_name` | 中文名称 | 标注范围与说明 |
|---:|---|---|---|
| 0 | `red_ball` | 红色球 | 仅标注画面中可见的红色球体；悬挂绳、拉线和其他附件不属于此类。 |
| 1 | `blue_ball` | 蓝色球 | 仅标注画面中可见的蓝色球体；悬挂绳、拉线和其他附件不属于此类。 |
| 2 | `door` | 门 | 每扇门单独标注一个实例，包含可见的门框/门体区域；不同高度的门共用此 ID。 |

## `classes.txt` 顺序

训练 8029 数据集时，`classes.txt` 必须严格写成：

```text
red_ball
blue_ball
door
```

对应关系如下：

| `classes.txt` 行号 | YOLO ID | `class_name` |
|---:|---:|---|
| 1 | 0 | `red_ball` |
| 2 | 1 | `blue_ball` |
| 3 | 2 | `door` |

## 标注约定

- 同类物体按实例分别标注；例如画面中有两扇门，就标注两个 `door` 实例。
- 红球和蓝球按球体颜色区分，不新增泛化的 `ball` 或其他颜色类别。
- 门的高低不单独拆分类别，统一使用 `door`。
- 水池、水面、池壁、灯光、网格背景以及其他非目标物体不作为类别标注。

## 与 8029 目录中现有标注文件的关系

`8029/frame_all_renamed/` 和 `8029/frame_all_renamed_with_label/` 中现有 JSON 的标签仍包含旧的 11 类名称，例如 `collection_frame_front`、`guide_line`、`target_rack_front` 等，与本表的 3 类定义不一致。

因此：

- 若按本表训练 8029 三类模型，必须先将标注转换/重新标注为 `red_ball`、`blue_ball`、`door`。
- 不能只替换 `classes.txt`，否则旧标注中的类别 ID 会被错误解释。
- `frame_all_renamed_no_label/` 仅表示无标注图片，不能据此推断类别 ID。
