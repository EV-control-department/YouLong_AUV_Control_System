"""Source YAML must win over stale installed/build configuration copies."""
from pathlib import Path

import pytest

from uv_task import config_loader as loader


@pytest.mark.parametrize('module', [
    'src/uv_task/uv_task/config_loader.py',
    'build/uv_task/uv_task/config_loader.py',
    'install/uv_task/lib/python3.8/site-packages/uv_task/config_loader.py',
    'install/lib/python3.12/site-packages/uv_task/config_loader.py',
])
def test_source_config_from_every_colcon_layout(tmp_path, monkeypatch, module):
    package = tmp_path / 'src/uv_task'
    config = package / 'config'
    (config / 'tasks').mkdir(parents=True)
    (config / 'missions').mkdir()
    (package / 'package.xml').write_text('<package/>')
    stale = tmp_path / 'install/uv_task/share/uv_task/config'
    stale.mkdir(parents=True)
    monkeypatch.setattr(loader, '__file__', str(tmp_path / module))
    assert loader.source_config_root() == config
    assert loader.default_mission_path() == config / 'missions/robocup_26.yaml'
    assert loader._resolve_task_config(Path('/tmp/custom/mission.yaml'), None, 'setz') == config / 'tasks/setz.yaml'
    assert loader._resolve_task_config(Path('/tmp/custom/mission.yaml'), 'next.yaml', 'setz') == Path('/tmp/custom/next.yaml')


def test_missing_source_does_not_fall_back_to_installed_yaml(tmp_path, monkeypatch):
    stale = tmp_path / 'install/uv_task/share/uv_task/config'
    stale.mkdir(parents=True)
    monkeypatch.setattr(loader, '__file__', str(tmp_path / 'install/uv_task/lib/python3.8/site-packages/uv_task/config_loader.py'))
    with pytest.raises(loader.ConfigError, match='src/uv_task/config'):
        loader.default_mission_path()


def test_source_edits_are_visible_without_rebuild(tmp_path, monkeypatch):
    package = tmp_path / 'src/uv_task'
    config = package / 'config/tasks'
    config.mkdir(parents=True)
    (package / 'package.xml').write_text('<package/>')
    monkeypatch.setattr(loader, '__file__', str(tmp_path / 'install/lib/python3.8/site-packages/uv_task/config_loader.py'))
    path = loader._resolve_task_config(tmp_path / 'mission.yaml', None, 'setz')
    path.write_text('task: setz\nparams: {}\n')
    assert loader._read_yaml(path)['params'] == {}
    path.write_text('task: setz\nparams: {depth: 1.2}\n')
    assert loader._read_yaml(path)['params']['depth'] == 1.2
