"""Local-only mock ActionServer: no hardware commands or vehicle nodes."""
import os
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uv_msgs.action import BasicMotion
from uv_task.action_client_compat import ReliableActionClient


pytestmark = pytest.mark.skipif(
    os.environ.get('UV_TASK_RUN_ROS_TESTS') != '1',
    reason='opt in to the isolated mock ROS exchange with UV_TASK_RUN_ROS_TESTS=1')


def ready(future, timeout=3.):
    deadline=time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline:
        time.sleep(.001)
    assert future.done(),'Action response did not complete'
    return future.result()


def test_fast_goals_results_and_cancellation_from_task_thread(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '188')
    rclpy.init()
    server_node=Node('compat_mock_motion');client_node=Node('compat_test_client')
    executor=MultiThreadedExecutor(num_threads=4)
    server=client=thread=None
    errors=[];completed=[]
    try:
        def execute(handle):
            result=BasicMotion.Result()
            if handle.request.target[0] > 0.:
                until=time.monotonic()+3.
                while not handle.is_cancel_requested and time.monotonic()<until:
                    time.sleep(.001)
                assert handle.is_cancel_requested
                handle.canceled();result.success=False;result.message='cancelled'
            else:
                handle.succeed();result.success=True
            return result
        server=ActionServer(server_node,BasicMotion,'/compat_test_motion',execute,
                            cancel_callback=lambda _:CancelResponse.ACCEPT,
                            callback_group=ReentrantCallbackGroup())
        client=ReliableActionClient(client_node,BasicMotion,'/compat_test_motion')
        executor.add_node(server_node);executor.add_node(client_node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        assert client.wait_for_server(timeout_sec=3.)
        def task():
            try:
                for _ in range(100):
                    goal=BasicMotion.Goal();goal.cmd_type=7;goal.target=[0.]*4
                    handle=ready(client.send_goal_async(goal))
                    assert handle.accepted
                    result=ready(handle.get_result_async())
                    assert result.status==4 and result.result.success
                    completed.append(True)
                goal.target=[1.,0.,0.,0.]
                handle=ready(client.send_goal_async(goal));assert handle.accepted
                result=handle.get_result_async()
                assert ready(handle.cancel_goal_async()).goals_canceling
                assert ready(result).status==5
            except Exception as error:errors.append(error)
        worker=threading.Thread(target=task,daemon=True);worker.start();worker.join(15.)
        assert not worker.is_alive() and not errors
        assert len(completed)==100
    finally:
        executor.shutdown(timeout_sec=3.)
        if thread is not None:thread.join(3.)
        if client is not None:client.destroy()
        if server is not None:server.destroy()
        client_node.destroy_node();server_node.destroy_node();rclpy.shutdown()
