"""A fast server can reply while the sending thread is still registering it."""
import ast
import asyncio
from concurrent.futures import Future
from pathlib import Path
from queue import Queue
import threading

import pytest

ROOT = Path(__file__).parents[1]


class OldActionClient:
    """Model the pre-fix rclpy ordering at the DDS/sequence-number boundary."""
    def __init__(self, *args, **kwargs):
        self.queue = Queue()
        self.pending = {}
        self.sent = threading.Event()
        self.release_sender = threading.Event()
        self.taken = threading.Event()
        self.warnings = []
        self.feedback = None
    def request(self, kind):
        self.queue.put((kind, 1, 'accepted'))
        self.sent.set()
        assert self.release_sender.wait(1.)
        future = Future()
        self.pending[kind, 1] = future
        future.add_done_callback(lambda _:self.pending.pop((kind, 1), None))
        return future
    def send_goal_async(self, *args, **kwargs):
        return self.request('goal')
    def _cancel_goal_async(self, *args, **kwargs):
        return self.request('cancel')
    def _get_result_async(self, *args, **kwargs):
        return self.request('result')
    def take_data(self):
        value = self.queue.get(timeout=1.)
        self.taken.set()
        return value
    async def execute(self, data):
        kind, sequence, response = data
        if kind == 'feedback':
            await self.feedback()
            return
        future = self.pending.get((kind, sequence))
        if future is None:
            self.warnings.append('Ignoring unexpected '+kind+' response')
        else:
            future.set_result(response)


def protected_class(base=OldActionClient):
    path = ROOT/'uv_task/action_client_compat.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    ns = dict(ActionClient=base, threading=threading, __name__=__name__)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),ns)
    return ns['ReliableActionClient']


@pytest.mark.parametrize('kind',['goal','cancel','result'])
@pytest.mark.parametrize('protected',[False,True])
def test_fast_reply_during_request_registration(kind,protected):
    client = (protected_class() if protected else OldActionClient)()
    returned, errors = [], []
    receiver_started = threading.Event()
    method = dict(goal=client.send_goal_async,cancel=client._cancel_goal_async,result=client._get_result_async)[kind]
    def send():
        try:returned.append(method(object()))
        except Exception as error:errors.append(error)
    def receive():
        try:
            receiver_started.set()
            asyncio.run(client.execute(client.take_data()))
        except Exception as error:errors.append(error)
    sender=threading.Thread(target=send);receiver=threading.Thread(target=receive)
    sender.start()
    try:
        assert client.sent.wait(1.)
        receiver.start();assert receiver_started.wait(1.)
        if protected:
            assert not client.taken.wait(.05)
        else:
            assert client.taken.wait(1.)
            receiver.join(1.);assert not receiver.is_alive()
    finally:
        client.release_sender.set();sender.join(1.)
        if receiver.ident is not None:receiver.join(1.)
    assert not sender.is_alive() and not receiver.is_alive() and not errors
    assert returned[0].done()==protected
    assert bool(client.warnings)!=protected


def test_real_unexpected_duplicate_reply_is_not_silenced():
    client=protected_class()();client.release_sender.set()
    future=client.send_goal_async(object());response=client.take_data()
    asyncio.run(client.execute(response))
    assert future.result()=='accepted'
    asyncio.run(client.execute(response))
    assert client.warnings==['Ignoring unexpected goal response']


def test_lock_exists_before_client_is_registered_with_executor():
    class RegisteringClient(OldActionClient):
        def __init__(self):
            assert hasattr(self,'_request_registration_lock')
            super().__init__()
    protected_class(RegisteringClient)()


def test_each_action_client_has_its_own_registration_lock():
    cls=protected_class();first,second=cls(),cls()
    assert first._request_registration_lock is not second._request_registration_lock


def test_async_feedback_does_not_hold_request_lock_across_await():
    client=protected_class()();client.release_sender.set()
    errors=[]
    async def run():
        loop=asyncio.get_running_loop();reply=loop.create_future()
        def send():
            try:
                client.send_goal_async(object())
                loop.call_soon_threadsafe(reply.set_result,True)
            except Exception as error:errors.append(error)
        async def feedback():
            worker=threading.Thread(target=send)
            worker.start()
            try:assert await asyncio.wait_for(reply,.5)
            finally:worker.join(1.)
        client.feedback=feedback
        await client.execute(('feedback',0,None))
    asyncio.run(run())
    assert not errors


def test_task_runner_uses_protected_client_for_all_motion_callers():
    tree=ast.parse((ROOT/'uv_task/task_runner.py').read_text())
    constructor=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(
        isinstance(t,ast.Attribute) and t.attr=='_action_client' for t in n.targets))
    assert constructor.value.func.id=='ReliableActionClient'
