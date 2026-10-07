"""Low-rate Linux process-tree samples, without importing ROS or image libraries."""

import os
from pathlib import Path
import time


class ProcessSampler:
    def __init__(self, root_pid, proc_root='/proc'):
        self.root_pid = root_pid
        self.proc = Path(proc_root)
        self.previous = {}
        self.ticks = os.sysconf('SC_CLK_TCK')
        self.page_size = os.sysconf('SC_PAGE_SIZE')

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
                           'threads': int(fields[17])})
        self.previous = previous
        return {'time_unix_ns': time.time_ns(), 'load_average': os.getloadavg(),
                'cpu_count': os.cpu_count(), 'processes': result}
