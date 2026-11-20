#!/usr/bin/env python3
"""Per-session detail of pack cell-voltage spread for the 4th set of runs.

Usage:
    python spread_detail.py

Takes no arguments. For each of LOG0063..LOG0066 it reads the decoded
<session>_slow.csv (min/max/avg cell mV) and <session>_fast.csv (bus current)
written by decode_vcu_log.py, and prints the spread median/max, how far the
lowest and highest cells sit from the average, first-quarter vs last-quarter
spread, the bus current range, and spread at idle (<3 A) vs load (>100 A).
Output goes to stdout only.

Hard-coded paths: the session list (LOG0063, LOG0064, LOG0065, LOG0066) and
the folder os.path.join("tools", "4th Set of runs"), relative to the current
working directory.
"""
import csv, os, statistics, bisect

IDLE_CURRENT_A = 3.0       # |bus current| below this counts as idle
LOADED_CURRENT_A = 100.0   # |bus current| above this counts as loaded
SESSIONS = ["LOG0063", "LOG0064", "LOG0065", "LOG0066"]
RUN_FOLDER = os.path.join("tools", "4th Set of runs")
QUARTER_DIVISOR = 4        # first/last quarter of the run for the trend

def read_csv_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))
def parse_float_or_none(x):
    try: return float(x)
    except: return None

for sess in SESSIONS:
    root = RUN_FOLDER
    slow = read_csv_rows(os.path.join(root, sess + "_slow.csv"))
    fast = read_csv_rows(os.path.join(root, sess + "_fast.csv"))
    rows = [r for r in slow if r.get("cell_v_valid") == "1"]
    pts = []
    for r in rows:
        vmn, vmx, vav, t = parse_float_or_none(r["min_cell_mv"]), parse_float_or_none(r["max_cell_mv"]), parse_float_or_none(r["avg_cell_mv"]), parse_float_or_none(r["t_ms"])
        if vmn and vmx and vmn > 0:
            pts.append((t, vmx-vmn, vmn, vmx, vav))
    if not pts:
        print(sess, "no valid cell data"); continue

    cur_t, cur_i = [], []
    for fr in fast:
        t, i = parse_float_or_none(fr["t_ms"]), parse_float_or_none(fr["bus_current_a"])
        if t is not None and i is not None: cur_t.append(t); cur_i.append(i)
    def current_at(t):
        if not cur_t: return None
        j = max(0, bisect.bisect_right(cur_t, t)-1); return cur_i[j]

    spreads = [p[1] for p in pts]
    low_dev = [p[4]-p[2] for p in pts]   # avg - min  (how far the lowest cell sits below mean)
    high_dev = [p[3]-p[4] for p in pts]  # max - avg
    # within-session trend: first 25% vs last 25% of time
    pts_sorted = sorted(pts)
    q = max(1, len(pts_sorted)//QUARTER_DIVISOR)
    first = statistics.median([p[1] for p in pts_sorted[:q]])
    last  = statistics.median([p[1] for p in pts_sorted[-q:]])
    imax = max(abs(i) for i in cur_i) if cur_i else 0
    idle = [p[1] for p in pts if current_at(p[0]) is not None and abs(current_at(p[0]))<IDLE_CURRENT_A]
    loaded   = [p[1] for p in pts if current_at(p[0]) is not None and abs(current_at(p[0]))>LOADED_CURRENT_A]

    print(f"== {sess} ==")
    print(f"  spread mV: median={statistics.median(spreads):.0f}  max={max(spreads):.0f}")
    print(f"  low-side dev (avg-min): median={statistics.median(low_dev):.0f}   high-side dev (max-avg): median={statistics.median(high_dev):.0f}")
    print(f"  within-run: first-quarter median={first:.0f} mV  ->  last-quarter median={last:.0f} mV")
    print(f"  bus current range: {min(cur_i):.0f}..{max(cur_i):.0f} A  (|max|={imax:.0f})")
    if idle: print(f"  spread @ idle (<3A):  median={statistics.median(idle):.0f} mV  (n={len(idle)})")
    if loaded:   print(f"  spread @ load(>100A): median={statistics.median(loaded):.0f} mV  (n={len(loaded)})")
    print()
