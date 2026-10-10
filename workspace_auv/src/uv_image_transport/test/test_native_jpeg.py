"""Optional cross-process integration against the deployed native binding."""

import multiprocessing
import uuid

import cv2
import numpy as np
import pytest

from uv_image_transport import ENCODING_JPEG, FrameHeader, Iceoryx2Publisher, Iceoryx2Reader


def _publish(service, frames, connection):
    try:
        with Iceoryx2Publisher(service) as publisher:
            for header, payload in frames:
                publisher.publish(header, payload)
                connection.send('sent')
                assert connection.recv() == 'received'
    except Exception as error:
        connection.send(repr(error))
        raise
    finally:
        connection.close()


def test_cross_process_jpeg_and_allocation_growth():
    pytest.importorskip('iceoryx2')
    frames = []
    for sequence, shape in enumerate(((16, 32), (1024, 1024), (32, 64))):
        image = np.random.default_rng(sequence).integers(0, 256, (*shape, 3), dtype=np.uint8)
        payload = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 100])[1].tobytes()
        frames.append((FrameHeader(sequence + 1, 1_000_000_000 + sequence, sequence + 1,
                                   1, shape[1], shape[0], 0, encoding=ENCODING_JPEG), payload))
    assert len(frames[1][1]) > 1024 * 1024
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe()
    service = 'youlong/test/jpeg/' + uuid.uuid4().hex
    process = context.Process(target=_publish, args=(service, frames, child))
    with Iceoryx2Reader(service) as reader:
        process.start()
        try:
            for header, payload in frames:
                assert parent.poll(10), 'native publisher did not send a frame'
                assert parent.recv() == 'sent'
                received = reader.read()
                assert received.header == header
                assert received.payload == payload
                parent.send('received')
            process.join(timeout=10)
            assert process.exitcode == 0
        finally:
            if process.is_alive():
                process.terminate()
                process.join(timeout=5)
            parent.close()
            child.close()
