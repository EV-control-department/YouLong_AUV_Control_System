#!/usr/bin/env python3
"""Split recorded front/down stereo JPEGs and package 200 images per ZIP.

Usage:
    python3 scripts/package_stereo_datasets.py
    python3 scripts/package_stereo_datasets.py --input records/datasets --output records/my_zips
"""

import argparse
from datetime import datetime
from io import BytesIO
from pathlib import Path
from zipfile import ZIP_STORED, ZipFile

from PIL import Image


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png'}


def source_images(root):
    """Visit every recorded session, including nested front/down folders."""
    return sorted(
        path for path in root.rglob('*')
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        and any(part in ('front', 'down')
                for part in path.relative_to(root).parts[:-1])
    )


def archive_name(root, source, eye):
    """Keep the full source path in a flat, unique image name."""
    relative = source.relative_to(root)
    return '__'.join((*relative.parts[:-1], relative.stem, eye)) + '.jpg'


def package(root, output, batch_size):
    if batch_size <= 0 or batch_size % 2:
        raise ValueError('batch size must be a positive even number to keep stereo pairs together')
    root = root.resolve()
    output = output.resolve()
    if not root.is_dir():
        raise FileNotFoundError(root)
    if output == root or root in output.parents:
        raise ValueError('output must be outside the source dataset tree')
    sources = source_images(root)
    if not sources:
        raise ValueError(f'no front/down images found under {root}')
    if output.exists():
        raise FileExistsError(f'output already exists: {output}')
    output.mkdir(parents=True)

    archive = None
    names = set()
    written = 0
    try:
        for source in sources:
            with Image.open(source) as image:
                image = image.convert('RGB')
                width, height = image.size
                if width < 2 or width % 2:
                    raise ValueError(f'expected an even-width stereo image: {source} ({width}x{height})')
                half = width // 2
                for eye, bounds in (
                    ('left', (0, 0, half, height)),
                    ('right', (half, 0, width, height)),
                ):
                    if written % batch_size == 0:
                        if archive is not None:
                            archive.close()
                        number = written // batch_size + 1
                        archive = ZipFile(output / f'dataset_{number:04d}.zip', 'x',
                                          compression=ZIP_STORED)
                    name = archive_name(root, source, eye)
                    if name in names:
                        raise ValueError(f'duplicate output name: {name}')
                    names.add(name)
                    buffer = BytesIO()
                    image.crop(bounds).save(buffer, format='JPEG', quality=95,
                                            subsampling=0)
                    archive.writestr(name, buffer.getvalue())
                    written += 1
    finally:
        if archive is not None:
            archive.close()
    return len(sources), written, sorted(output.glob('dataset_*.zip'))


def main():
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path,
                        default=repository / 'records/datasets')
    parser.add_argument('--output', type=Path,
                        default=repository / 'records' /
                        f'dataset_zips_{datetime.now():%Y%m%d_%H%M%S}')
    parser.add_argument('--batch-size', type=int, default=200,
                        help='number of split images per ZIP (even; default: 200)')
    args = parser.parse_args()
    sources, images, archives = package(args.input, args.output, args.batch_size)
    print(f'{sources} stereo frames -> {images} eye images -> {len(archives)} ZIP files')
    for path in archives:
        print(path)


if __name__ == '__main__':
    main()
