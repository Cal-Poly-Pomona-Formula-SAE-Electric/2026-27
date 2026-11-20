#!/usr/bin/env python3
"""Quantify pack cell-voltage spread and test the bad-wire-bond hypothesis.

Spread = max_cell_mv - min_cell_mv (from the SLOW records).
A high-resistance interconnect (bad weld/bond) makes the affected cell sag under
discharge, so spread should grow with |bus current| and shrink at rest. A real
chemistry imbalance shows spread even at idle. We test this by:
  1. Spread distribution per session (median, 95th pct, idle vs loaded).
  2. Correlation of spread with bus current (joined from FAST records by time).

Reads: decoded *_slow.csv (and the matching *_fast.csv) from the folders
tools/, tools/3rd set of runs/ and tools/4th Set of runs/, relative to the
current directory (hard-coded in SEARCH_FOLDERS).
Writes: nothing, prints one table row per session to stdout.
Run: python analyze_spread.py   (no arguments)
"""
import csv, glob, os, statistics, bisect

IDLE_CURRENT_A = 5.0       # |bus current| below this counts as idle
LOADED_CURRENT_A = 60.0    # |bus current| above this counts as loaded
MIN_PAIRS_FOR_CORRELATION = 5
P95_FRACTION = 0.95

def read_csv_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

def parse_float_or_none(x):
    try: return float(x)
    except: return None

def session_name(fast_path):
    base = os.path.basename(fast_path)
    return base.split("_")[0]

results = []
SEARCH_FOLDERS = ["tools", os.path.join("tools", "3rd set of runs"),
                  os.path.join("tools", "4th Set of runs")]
for root in SEARCH_FOLDERS:
    for slow_path in sorted(glob.glob(os.path.join(root, "*_slow.csv"))):
        stem = slow_path[:-len("_slow.csv")]
        fast_path = stem + "_fast.csv"
        slow = read_csv_rows(slow_path)
        # valid cell-voltage rows only
        rows = [r for r in slow if r.get("cell_v_valid") == "1"]
        spreads = []
        for r in rows:
            vmn, vmx = parse_float_or_none(r["min_cell_mv"]), parse_float_or_none(r["max_cell_mv"])
            if vmn and vmx and vmn > 0:
                spreads.append((parse_float_or_none(r["t_ms"]), vmx - vmn, vmn, vmx, parse_float_or_none(r["avg_cell_mv"])))
        if not spreads:
            continue

        # Join bus current from FAST by nearest earlier timestamp
        cur_t, cur_i = [], []
        if os.path.exists(fast_path):
            for fr in read_csv_rows(fast_path):
                t, i = parse_float_or_none(fr["t_ms"]), parse_float_or_none(fr["bus_current_a"])
                if t is not None and i is not None:
                    cur_t.append(t); cur_i.append(i)

        def current_at(t):
            if not cur_t: return None
            j = bisect.bisect_right(cur_t, t) - 1
            if j < 0: j = 0
            return abs(cur_i[j])

        sp_vals = [s[1] for s in spreads]
        idle = [s[1] for s in spreads if current_at(s[0]) is not None and current_at(s[0]) < IDLE_CURRENT_A]
        loaded = [s[1] for s in spreads if current_at(s[0]) is not None and current_at(s[0]) > LOADED_CURRENT_A]

        # correlation spread vs |current|
        pairs = [(current_at(s[0]), s[1]) for s in spreads if current_at(s[0]) is not None]
        corr = None
        if len(pairs) > MIN_PAIRS_FOR_CORRELATION:
            xs = [p[0] for p in pairs]; ys = [p[1] for p in pairs]
            try: corr = statistics.correlation(xs, ys)
            except: corr = None

        results.append({
            "sess": session_name(slow_path),
            "n": len(sp_vals),
            "median": statistics.median(sp_vals),
            "p95": sorted(sp_vals)[int(P95_FRACTION*len(sp_vals))-1],
            "max": max(sp_vals),
            "idle_med": statistics.median(idle) if idle else None,
            "load_med": statistics.median(loaded) if loaded else None,
            "corr": corr,
        })

print(f"{'sess':<10}{'n':>5}{'med':>6}{'p95':>6}{'max':>6}{'idle':>6}{'load':>6}{'corr':>7}")
for r in results:
    idle = f"{r['idle_med']:.0f}" if r['idle_med'] is not None else "-"
    load = f"{r['load_med']:.0f}" if r['load_med'] is not None else "-"
    corr = f"{r['corr']:+.2f}" if r['corr'] is not None else "-"
    print(f"{r['sess']:<10}{r['n']:>5}{r['median']:>6.0f}{r['p95']:>6.0f}{r['max']:>6.0f}{idle:>6}{load:>6}{corr:>7}")
