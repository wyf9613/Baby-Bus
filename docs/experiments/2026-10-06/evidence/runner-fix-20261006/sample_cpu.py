import json
import os
from pathlib import Path
import sys
import time

output = Path(sys.argv[1])
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
hz = os.sysconf('SC_CLK_TCK')
previous = {}
samples = []
start = time.monotonic()
while time.monotonic() - start < duration:
    now = time.monotonic()
    processes = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if not any(name in command for name in ('dream_command.runtime_runner', '/policy_node.py', '/traxxas_vehicle_interface --ros-args')):
                continue
            stat = (entry / 'stat').read_text().rpartition(') ')[2].split()
            ticks = int(stat[11]) + int(stat[12])
            born = int(stat[19])
        except (OSError, ValueError, IndexError):
            continue
        key = (entry.name, born)
        row = {'pid': int(entry.name), 'command': command, 'cpu_ticks': ticks}
        if key in previous:
            old_time, old_ticks = previous[key]
            row['cpu_percent'] = (ticks-old_ticks)/hz/(now-old_time)*100
        previous[key] = (now, ticks)
        processes.append(row)
    samples.append({'elapsed_s': now-start, 'processes': processes})
    time.sleep(1)
output.write_text(json.dumps(samples, indent=2)+'\n')
for keyword in ('dream_command.runtime_runner', '/policy_node.py', '/traxxas_vehicle_interface --ros-args'):
    values = [p['cpu_percent'] for s in samples for p in s['processes'] if keyword in p['command'] and 'cpu_percent' in p]
    if values:
        print(keyword, 'mean CPU %', round(sum(values)/len(values), 2), 'max', round(max(values), 2))
