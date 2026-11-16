"""JSON Lines run log for Task 2 and a summary of a finished run. No ROS dependency.

Each line is one record: {"t": seconds since mission start (sim time), "event": ..., ...}.
Events: start, plan, replan, replan_stale, goal_sent, arrived, wait_partner, wait_precedence,
dwell_start, dwell_reset, rescued, failed, resend, late, dropped, skipped, rules, rules_rejected,
distance (1 Hz robot-to-target distance), time_limit, finished.
"""
import datetime
import json
import os


class RunLog:
    def __init__(self, log_dir, prefix='task2'):
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.abspath(os.path.join(log_dir, f'{prefix}_{stamp}.jsonl'))
        self._f = open(self.path, 'a', buffering=1)  # line buffered: a crash keeps everything so far

    def write(self, record):
        self._f.write(json.dumps(record, default=_jsonable) + '\n')

    def close(self):
        self._f.close()


def _jsonable(value):
    if isinstance(value, (set, tuple)):
        return list(value)
    return str(value)


def read_log(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def summarize(records):
    """Human-readable summary lines of one run."""
    lines = []
    start = next((r for r in records if r['event'] == 'start'), None)
    if start:
        lines.append(f"robots: {', '.join(start['robots'])}; targets: "
                     + ', '.join(f"{t['id']} ({t['type']})" for t in start['targets'])
                     + (f"; rules: {start['precedence']}" if start['precedence'] else ''))
    plans = [r for r in records if r['event'] == 'plan']
    for p in plans:
        lines.append(f"plan @ {p['t']:.1f} s: {p['status']} ({p['mode']}), solved in {p['solve_time']:.2f} s, "
                     f"planned makespan {p['makespan']:.1f} s")
        for robot, route in p['routes'].items():
            lines.append(f"    {robot}: " + (' -> '.join(f"{v['task']}[{v['slot']}] {v['start']:.1f}-{v['end']:.1f}"
                                                       for v in route) or 'idle'))
    replans = [r for r in records if r['event'] == 'replan']
    adopted = [r for r in replans if r['adopted']]
    if replans:
        lines.append(f"replans: {len(replans)} ({len(adopted)} adopted); reasons: "
                     + ', '.join(sorted({r['reason'] for r in replans})))
        times = [r['solve_time'] for r in replans]
        lines.append(f"replan solve time: max {max(times):.2f} s, mean {sum(times) / len(times):.2f} s")
    lines.append('rescues:')
    for r in records:
        if r['event'] == 'rescued':
            lines.append(f"    {r['task']} by {', '.join(r['robots'])}: dwell {r['start']:.1f}-{r['end']:.1f} s "
                         f"({r['end'] - r['start']:.1f} s continuous), planned end {r['planned_end']:.1f} s, "
                         f"distances {r['d']}")
    for event in ('dwell_reset', 'failed', 'dropped', 'late', 'resend'):
        hits = [r for r in records if r['event'] == event]
        if hits:
            lines.append(f"{event}: {len(hits)} - " + '; '.join(
                f"{h['t']:.1f}s {h.get('task', '')} {h.get('robot', '')} {h.get('reason', '')}".strip()
                for h in hits))
    fin = next((r for r in records if r['event'] == 'finished'), None)
    if fin:
        lines.append(f"finished: rescued {fin['rescued']}, dropped {fin['dropped']}; actual makespan "
                     f"{fin['makespan']:.1f} s vs planned {fin['planned_makespan']} s")
    else:
        lines.append('run did not finish (no "finished" record)')
    return lines
