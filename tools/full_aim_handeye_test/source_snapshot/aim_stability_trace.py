"""Small independent bounded logger for optional aimer/preview observations."""
from collections import Counter
import csv
import json
import os
from pathlib import Path
import time


class BufferedTrace:
    def __init__(self, path, capacity=100000):
        self.path = Path(path)
        if self.path.exists() or self.path.with_suffix('.metadata.json').exists():
            raise FileExistsError('Use a new trace path')
        self.capacity = capacity
        self.rows = []
        self.counts = Counter()
        self.started_ns = time.monotonic_ns()
        self.closed = False

    def emit(self, event, **fields):
        self.counts[event] += 1
        if len(self.rows) >= self.capacity:
            self.counts['log_dropped'] += 1
            return None
        row = dict(event=event, host_ns=time.monotonic_ns(), **fields)
        self.rows.append(row)
        return row

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.path.parent.mkdir(parents=True, exist_ok=True)
        keys = list(dict.fromkeys(k for row in self.rows for k in row)) or ['event','host_ns']
        with self.path.open('x', newline='') as stream:
            writer = csv.DictWriter(stream,fieldnames=keys)
            writer.writeheader()
            writer.writerows({k:json.dumps(v) if isinstance(v,(list,tuple,dict)) else v
                              for k,v in row.items()} for row in self.rows)
        self.path.with_suffix('.metadata.json').write_text(json.dumps(dict(
            clock='time.monotonic_ns',unit='ns',
            resolution_s=time.get_clock_info('monotonic').resolution,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            time_namespace=os.readlink('/proc/self/ns/time'),counts=dict(self.counts),
            started_ns=self.started_ns,stopped_ns=time.monotonic_ns()),indent=2)+'\n')
