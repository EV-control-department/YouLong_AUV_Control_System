"""Bounded observation window for a future smoother/backend."""

from collections import deque


class SlidingWindow:
    def __init__(self, size=20):
        self._items = deque(maxlen=max(1, int(size)))

    def append(self, item):
        self._items.append(item)

    def snapshot(self):
        return tuple(self._items)
