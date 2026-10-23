"""Use the shared session recorder with only the structured-light image source."""

from uv_record.recorder import _parse_args as shared_parse_args, run_recorder
from uv_record.session import project_root


def default_output_root():
    return project_root() / 'records' / 'sl_sessions'


def _parse_args(argv=None):
    args = shared_parse_args(
        argv, output_root=default_output_root(), video_fps=30.0,
        record_modes=('raw',), prog='sl_record record')
    args.raw_cameras = ('sl',)
    return args


def main():
    return run_recorder(_parse_args())
