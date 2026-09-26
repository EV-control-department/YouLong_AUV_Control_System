"""Deprecated compatibility re-exports; import from auv_protocol.topics."""

from auv_protocol.topics import (
    MAPPING_KEYFRAMES,
    MAPPING_LANDMARKS,
    MAPPING_LOCALIZATION_OPPORTUNITY,
)

__all__ = [
    'MAPPING_LANDMARKS', 'MAPPING_KEYFRAMES',
    'MAPPING_LOCALIZATION_OPPORTUNITY',
]
