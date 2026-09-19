"""Stonefish stereo camera to canonical camera transport adapter."""

# The implementation is kept in the compatibility module for one release so
# old imports continue to work.  The runtime bridge uses this named adapter
# class, making the ownership boundary explicit.
from .camera_passthrough import CameraPassthrough


class CameraAdapter(CameraPassthrough):
    """Republish raw eyes and synchronized stitched camera frames."""

    raw_topic_prefix = '/auv/sim/raw/camera/'
    canonical_topic_prefix = '/auv/sensors/camera/'


__all__ = ['CameraAdapter']
