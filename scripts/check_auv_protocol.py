"""Static protocol and Ground Truth isolation checks for CI."""

from __future__ import annotations

import ast
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
CODE_ROOTS = (ROOT / 'workspace_auv' / 'src', ROOT / 'workspace_sim' / 'src')
GT_ALLOWED = {'auv_protocol', 'uv_sim_evaluation', 'uv_sim_bringup'}


def python_files():
    for code_root in CODE_ROOTS:
        if code_root.is_dir():
            yield from code_root.rglob('*.py')


def check_topics():
    errors = []
    for path in python_files():
        if '/test/' in str(path):
            continue
        try:
            tree = ast.parse(path.read_text(encoding='utf-8'))
        except SyntaxError as error:
            errors.append(f'{path}: syntax error: {error}')
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                value = node.value
                if value.startswith(('/cameras/', '/sensors/', '/state/',
                                     '/control/', '/sim/', '/zit6/',
                                     '/task/')):
                    if 'legacy' not in path.name.lower() and 'protocol' not in str(path):
                        errors.append(f'{path}:{node.lineno}: unscoped topic {value}')
        if 'SIM_GT_ODOM' in path.read_text(encoding='utf-8'):
            package = path.parts[path.parts.index('src') + 1]
            if package not in GT_ALLOWED:
                errors.append(f'{path}: Ground Truth symbol used outside evaluation/sim boundary')
    return errors


def check_workspace_boundary():
    """Keep the real AUV workspace independent of Stonefish packages."""
    errors = []
    auv_root = ROOT / 'workspace_auv' / 'src'
    for package_xml in auv_root.glob('*/package.xml'):
        source = package_xml.read_text(encoding='utf-8')
        for forbidden in ('<exec_depend>uv_sim',
                          '<exec_depend>stonefish_ros2',
                          '<depend>uv_sim',
                          '<depend>stonefish_ros2'):
            if forbidden in source:
                errors.append(
                    f'{package_xml}: AUV workspace depends on {forbidden}')
    return errors


def main():
    errors = check_topics() + check_workspace_boundary()
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print('AUV protocol check: PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
