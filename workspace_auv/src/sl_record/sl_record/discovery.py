"""ROS subscription boundary for a catalog-driven image archive."""

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from sl_camera.catalog import catalog_qos
from sl_msgs.msg import CameraCatalog


class CatalogSubscriber(Node):
    def __init__(self, frames, topic, **kwargs):
        super().__init__('sl_record_discovery', **kwargs)
        self.frames = frames
        self.subscription = self.create_subscription(
            CameraCatalog, topic, frames.on_catalog, catalog_qos())


class DiscoveryThread:
    def __init__(self, frames, topic):
        self.frames = frames
        self.stop_event = threading.Event()
        self.node = CatalogSubscriber(frames, topic)
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._run, name='sl-record-discovery', daemon=True)

    def _run(self):
        try:
            while not self.stop_event.is_set() and rclpy.ok(context=self.node.context):
                self.executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            pass
        except Exception as error:
            self.frames.error = error

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise TimeoutError('DDS discovery thread did not stop')
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()
