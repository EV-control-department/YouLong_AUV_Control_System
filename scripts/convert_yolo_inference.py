"""Convert a trusted YOLO segmentation checkpoint to inference-only weights.

PyTorch checkpoints execute pickle code: use only your own trusted model.
Missing training loss classes are restored temporarily, then removed. Unknown
network layers are not substituted; original weights are never overwritten.
"""
import argparse
import copy
import pickle
import types
from pathlib import Path

import numpy as np
import torch
from ultralytics import YOLO


class TrainingLossPlaceholder(torch.nn.Module):
    def forward(self, *args, **kwargs):
        raise RuntimeError('Training-only placeholder must never execute')


class InferenceUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        # These two loss classes caused the 8.4.45 -> 8.3.14 load failure.
        # Do not mask missing model layers or arbitrary training classes.
        if module == 'ultralytics.utils.loss' and name in {
                'BCEDiceLoss', 'MultiChannelDiceLoss'}:
            try:
                return super().find_class(module, name)
            except AttributeError:
                print('临时恢复训练对象（不会用于推理）：', name)
                return TrainingLossPlaceholder
        return super().find_class(module, name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--device', default='cpu', help='Verification device, e.g. cuda:0')
    args = parser.parse_args()
    source = args.source.resolve()
    target = (args.output or source.with_name('last_inference.pt')).resolve()
    if target.exists() or target == source:
        parser.error('输出已存在或与原模型相同，拒绝覆盖')
    compat_pickle = types.ModuleType('inference_checkpoint_pickle')
    compat_pickle.__dict__.update(pickle.__dict__)
    compat_pickle.Unpickler = InferenceUnpickler
    checkpoint = torch.load(source, map_location='cpu', pickle_module=compat_pickle,
                            weights_only=False)
    model = copy.deepcopy(checkpoint.get('ema') or checkpoint['model'])
    for module in model.modules():
        if hasattr(module, 'criterion'):
            delattr(module, 'criterion')
    if any(isinstance(module, TrainingLossPlaceholder) for module in model.modules()):
        raise RuntimeError('训练占位对象仍在网络内部，拒绝生成推理权重')
    model.float().eval()
    # Drop optimizer/training state as well as criterion. Network parameters,
    # class names and architecture are preserved in the model itself.
    with target.open('xb') as stream:
        torch.save({'model': model, 'ema': None,
                    'train_args': checkpoint.get('train_args', {}),
                    'version': checkpoint.get('version'),
                    'date': checkpoint.get('date')}, stream)
    detector = YOLO(str(target))
    if detector.task != 'segment':
        raise RuntimeError('模型不是 segment；生成文件不得用于本次部署')
    for _ in range(3):
        result = detector.predict(np.zeros((480, 640, 3), dtype=np.uint8),
                                  device=args.device, verbose=False)[0]
    print('推理验证通过：', target)
    print('类别：', detector.names, '耗时(ms)：', result.speed)


if __name__ == '__main__':
    main()
