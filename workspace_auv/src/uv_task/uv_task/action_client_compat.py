"""Keep fast action responses behind request registration on older rclpy."""
import threading

from rclpy.action import ActionClient


class ReliableActionClient(ActionClient):
    """Serialize request bookkeeping and taking replies across task/executor threads.

    Foxy sends a DDS request before registering its sequence number. Its
    executor can otherwise take and discard a fast response in that gap
    (ros2/rclpy#1123). This also affects cancel and result requests.
    Keep the lock out of execute(): user feedback callbacks may await futures.
    """

    def __init__(self, *args, **kwargs):
        self._request_registration_lock = threading.RLock()
        super().__init__(*args, **kwargs)

    def send_goal_async(self, *args, **kwargs):
        with self._request_registration_lock:
            return super().send_goal_async(*args, **kwargs)

    def _cancel_goal_async(self, *args, **kwargs):
        with self._request_registration_lock:
            return super()._cancel_goal_async(*args, **kwargs)

    def _get_result_async(self, *args, **kwargs):
        with self._request_registration_lock:
            return super()._get_result_async(*args, **kwargs)

    def take_data(self):
        with self._request_registration_lock:
            return super().take_data()
