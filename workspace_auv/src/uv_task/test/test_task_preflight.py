"""No mission execution thread may start before complete YAML validation."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from uv_task.config_loader import ConfigError, load_mission_or_task


SOURCE = Path(__file__).parents[1] / 'uv_task/task_runner.py'


def extract(names, namespace):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'TaskRunnerNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace


@pytest.fixture
def node():
    events = []
    logger = NS(info=lambda text: events.append(('info', text)),
                error=lambda text: events.append(('error', text)))
    namespace = extract({'load_tasks'}, {'Path': Path, 'ConfigError': ConfigError,
                        'load_mission_or_task': load_mission_or_task})
    node = NS(_model_mapping=None, task_map={'setz': lambda p: None},
              events=events, get_logger=lambda: logger)
    node.load_tasks = lambda path: namespace['load_tasks'](node, path)
    return node


def mission(tmp_path, second='next.yaml'):
    (tmp_path / 'first.yaml').write_text('task: setz\nparams: {z: 1.0}\n')
    path = tmp_path / 'competition.yaml'
    path.write_text('mission:\n  name: test\n  tasks:\n'
                    '    - {name: setz, config: first.yaml}\n'
                    f'    - {{name: setz, config: {second}}}\n')
    return path


def test_later_missing_task_rejects_whole_mission_without_success_logs(node, tmp_path):
    path = mission(tmp_path)
    with pytest.raises(ConfigError, match='next.yaml'):
        node.load_tasks(path)
    assert any('不进入比赛' in text for _, text in node.events)
    assert not any('预检通过 [' in text for _, text in node.events)


def test_all_tasks_pass_before_success_and_snapshot_is_retained(node, tmp_path):
    path = mission(tmp_path)
    second = tmp_path / 'next.yaml'
    second.write_text('task: setz\nparams: {z: 2.0}\n')
    tasks = node.load_tasks(path)
    assert [task['params']['z'] for task in tasks] == [1., 2.]
    assert sum('预检通过 [' in text for _, text in node.events) == 2
    assert '全部通过' in node.events[-1][1]
    second.write_text('broken YAML: [')
    assert tasks[1]['params']['z'] == 2.0


def test_valid_yaml_without_runtime_handler_is_rejected(node, tmp_path):
    path = tmp_path / 'task.yaml'
    path.write_text('task: setz\nparams: {z: 1.0}\n')
    node.task_map = {}
    with pytest.raises(ConfigError, match='任务处理器'):
        node.load_tasks(path)


def test_invalid_encoding_is_a_configuration_error(node, tmp_path):
    path = tmp_path / 'task.yaml'
    path.write_bytes(b'\xff\xfe\xff')
    with pytest.raises(ConfigError, match='task.yaml'):
        node.load_tasks(path)
    assert '不进入比赛' in node.events[-1][1]


@pytest.mark.parametrize('failure', [True, False])
def test_service_never_starts_thread_before_preflight(node, failure):
    calls = []
    namespace = extract({'_run_task_cb'}, {
        'ConfigError': ConfigError, 'Parameter': lambda *a, **k: None,
        'TaskStatus': NS(STATUS_RUNNING=1),
        'threading': NS(Thread=lambda **kw: calls.append('thread') or
                        NS(start=lambda: calls.append('start'))),
    })
    node.running = node._debug_executing = False
    node._resolve_mission_path = lambda value: '/source/competition.yaml'
    node.set_parameters = lambda params: None
    node.run_task_list = lambda: None
    def load(path):
        calls.append('preflight')
        if failure:
            raise ConfigError('later task unreadable')
        return [{'name': 'setz'}]
    node.load_tasks = load
    response = namespace['_run_task_cb'](node, NS(start=True, task_name=''), NS())
    assert response.success is not failure
    assert calls == (['preflight'] if failure else ['preflight', 'thread', 'start'])


@pytest.mark.parametrize('failure', [True, False])
def test_auto_start_is_after_preflight_and_errors_exit_without_thread(failure):
    tree = ast.parse(SOURCE.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    events = []
    node = NS(_debug_mode=False, _auto_start=True, mission_file='', tasks=[],
              wait_for_model_mapping=lambda: True,
              _resolve_mission_path=lambda path: '/source/competition.yaml',
              get_logger=lambda: NS(info=lambda text: None, fatal=lambda text: None),
              destroy_node=lambda: events.append('destroy'),
              _stop_active_motion=lambda: None, run_task_list=lambda: None)
    def load(path):
        events.append('preflight')
        if failure:
            raise ConfigError('bad final task')
        return [{'name': 'setz'}]
    node.load_tasks = load
    ns = {'ConfigError': ConfigError, 'TaskRunnerNode': lambda: node,
          'rclpy': NS(init=lambda **kw: None, try_shutdown=lambda: None,
                      spin=lambda n: None, shutdown=lambda: None),
          'threading': NS(Thread=lambda **kw: events.append('thread') or
                          NS(start=lambda: events.append('start')))}
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(SOURCE), 'exec'), ns)
    if failure:
        with pytest.raises(SystemExit) as error:
            ns['main']()
        assert error.value.code == 2
        assert events == ['preflight', 'destroy']
    else:
        ns['main']()
        assert events[:3] == ['preflight', 'thread', 'start']
