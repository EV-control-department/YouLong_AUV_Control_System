"""Low-rate Linux process-tree samples, without importing ROS or image libraries."""

import os
from pathlib import Path
import time


class ProcessSampler:
    def __init__(self, root_pid, proc_root='/proc', storage_path=None):
        self.root_pid = root_pid
        self.proc = Path(proc_root)
        self.storage_path = Path(storage_path) if storage_path is not None else None
        self.previous = {}
        self.previous_cpu = None
        self.previous_disk = None
        self.ticks = os.sysconf('SC_CLK_TCK')
        self.page_size = os.sysconf('SC_PAGE_SIZE')

    @staticmethod
    def _process_io(path):
        try:
            values = {}
            for line in (path / 'io').read_text().splitlines():
                key, value = line.split(':', 1)
                if key in ('read_bytes', 'write_bytes', 'cancelled_write_bytes'):
                    values[key] = int(value.strip())
            return values
        except (OSError, ValueError):
            return None

    def _cpu_iowait_percent(self):
        try:
            fields = (self.proc / 'stat').read_text().splitlines()[0].split()
            if fields[0] != 'cpu' or len(fields) < 6:
                return None
            ticks = [int(value) for value in fields[1:9]]
            total = sum(ticks)
            iowait = ticks[4]
        except (OSError, IndexError, ValueError):
            return None
        previous = self.previous_cpu
        self.previous_cpu = (total, iowait)
        if previous is None:
            return None
        total_delta = total - previous[0]
        iowait_delta = iowait - previous[1]
        if total_delta <= 0 or iowait_delta < 0:
            return None
        return round(100.0 * iowait_delta / total_delta, 2)

    def _diskstats_for_storage_path(self):
        if self.storage_path is None:
            return None
        try:
            device = os.stat(self.storage_path).st_dev
            major, minor = os.major(device), os.minor(device)
            for line in (self.proc / 'diskstats').read_text().splitlines():
                fields = line.split()
                if len(fields) < 14 or int(fields[0]) != major or int(fields[1]) != minor:
                    continue
                values = [int(value) for value in fields[3:14]]
                return {
                    'name': fields[2],
                    'major': major,
                    'minor': minor,
                    'read_ios': values[0],
                    'read_sectors': values[2],
                    'read_ms': values[3],
                    'write_ios': values[4],
                    'write_sectors': values[6],
                    'write_ms': values[7],
                    'in_flight': values[8],
                    'io_ms': values[9],
                    'weighted_io_ms': values[10],
                }
        except (OSError, ValueError):
            return None
        return None

    def _storage_io_sample(self, now):
        current = self._diskstats_for_storage_path()
        if current is None:
            return {
                'available': False,
                'reason': 'target filesystem device is not present in /proc/diskstats',
            }
        previous = self.previous_disk
        self.previous_disk = (now, current)
        result = {
            'available': True,
            'device': current['name'],
            'major': current['major'],
            'minor': current['minor'],
            'in_flight': current['in_flight'],
        }
        if previous is None or previous[1]['name'] != current['name']:
            result['interval_seconds'] = None
            return result
        elapsed = now - previous[0]
        if elapsed <= 0:
            result['interval_seconds'] = None
            return result
        before = previous[1]
        deltas = {
            key: current[key] - before[key]
            for key in ('read_ios', 'read_sectors', 'read_ms', 'write_ios',
                        'write_sectors', 'write_ms', 'io_ms', 'weighted_io_ms')
        }
        elapsed_ms = elapsed * 1000.0
        result.update({
            'interval_seconds': round(elapsed, 3),
            'read_bytes_per_sec': round(max(0, deltas['read_sectors']) * 512 / elapsed, 1),
            'write_bytes_per_sec': round(max(0, deltas['write_sectors']) * 512 / elapsed, 1),
            'read_iops': round(max(0, deltas['read_ios']) / elapsed, 2),
            'write_iops': round(max(0, deltas['write_ios']) / elapsed, 2),
            'read_await_ms': (
                round(max(0, deltas['read_ms']) / deltas['read_ios'], 3)
                if deltas['read_ios'] > 0 else None),
            'write_await_ms': (
                round(max(0, deltas['write_ms']) / deltas['write_ios'], 3)
                if deltas['write_ios'] > 0 else None),
            'device_utilization_percent': round(
                100.0 * max(0, deltas['io_ms']) / elapsed_ms, 2),
            'average_queue_depth': round(
                max(0, deltas['weighted_io_ms']) / elapsed_ms, 3),
        })
        return result

    def sample(self):
        now = time.monotonic()
        processes = {}
        for path in self.proc.iterdir():
            if not path.name.isdigit():
                continue
            try:
                raw = (path / 'stat').read_text()
                fields = raw[raw.rfind(')') + 2:].split()
                processes[int(path.name)] = (raw[raw.find('(') + 1:raw.rfind(')')], fields)
            except (OSError, ValueError):
                continue
        selected = {self.root_pid}
        while True:
            children = {pid for pid, (_, fields) in processes.items()
                        if int(fields[1]) in selected}
            if children <= selected:
                break
            selected.update(children)
        result, previous = [], {}
        for pid in sorted(selected & processes.keys()):
            name, fields = processes[pid]
            ticks = int(fields[11]) + int(fields[12])
            identity = (pid, fields[19])  # start time guards against PID reuse
            before = self.previous.get(identity)
            cpu = None if before is None else max(
                0.0, 100.0 * (ticks - before[1]) / self.ticks / max(1e-6, now - before[0]))
            previous[identity] = (now, ticks)
            result.append({'pid': pid, 'name': name, 'cpu_percent': cpu,
                           'rss_bytes': int(fields[21]) * self.page_size,
                           'threads': int(fields[17]),
                           'io': self._process_io(self.proc / str(pid))})
        self.previous = previous
        return {'time_unix_ns': time.time_ns(), 'load_average': os.getloadavg(),
                'cpu_count': os.cpu_count(),
                'system_cpu_iowait_percent': self._cpu_iowait_percent(),
                'storage_io': self._storage_io_sample(time.monotonic()),
                'processes': result}
