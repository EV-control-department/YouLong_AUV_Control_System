"""Record every available structured-light/downward camera and ROS telemetry."""

import argparse

from sl_camera.profile import load_profile
from uv_record.recorder import _parse_args as shared_parse_args, run_recorder
from uv_record.session import project_root


def default_output_root():
    return project_root() / 'records' / 'sl_sessions'


def _parse_args(argv=None):
    selector = argparse.ArgumentParser(add_help=False)
    selector.add_argument('--profile', default='default')
    selected, _ = selector.parse_known_args(argv)
    profile = load_profile(selected.profile)

    def configure(parser):
        parser.add_argument('--profile', default='default',
                            help='shared collection profile name or YAML path')

    args = shared_parse_args(
        argv, output_root=profile.recording['output_root'] or default_output_root(),
        video_fps=profile.recording['video_fps'], record_modes=('raw',),
        prog='sl_record record', configure_parser=configure,
        defaults={key: profile.recording[key] for key in ('bag_duration', 'bag_storage')})
    args.profile = str(profile.path)
    args.raw_cameras = ('sl', 'down')
    args.optional_cameras = True
    return args


def main():
    return run_recorder(_parse_args())
