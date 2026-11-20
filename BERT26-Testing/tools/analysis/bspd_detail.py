#!/usr/bin/env python3
"""Show context around BSPD-low events and re-check torque calc above 1000 rpm.

Reads: VCULOG01 BIN files LOG0008, LOG0010, LOG0011, LOG0012 in the folder
"3rd set of runs/" (relative to the current directory; hard-coded below).
Writes: nothing, prints to stdout.
Run: python bspd_detail.py   (no arguments; run from the folder that holds
"3rd set of runs/")
"""
import struct, sys

HEADER_FMT = "<8sIBBBB"
FAST_FMT = "<BIHHHHHhhhhH"
SIZES = {1: struct.calcsize(FAST_FMT), 2: struct.calcsize("<BIhhhhhhHhhBBBBHIIBB"),
         3: struct.calcsize("<BIHHHhhhHBB"), 4: struct.calcsize("<BIhhhhhhh")}
RECORD_FAST = 1
DIVISOR_TENTHS = 10.0       # fixed-point 0.1 resolution (Nm, V, A)
MS_PER_S = 1000.0
W_PER_KW = 1000.0
NM_RPM_PER_KW = 9549.3      # T[Nm] = 9549.3 * P[kW] / n[rpm]
MIN_RPM_FOR_TORQUE = 1000
FLAGS = ["braking", "hard_braking", "bspc_active", "apps_plausible", "rtd_latched",
         "drive_mode_d", "bms_fault", "imd_fault", "pm100_fault", "bms_timeout",
         "imd_can_timeout", "contactors_closed", "bspd_gpio_ok", "imd_gpio_ok",
         "bms_gpio_ok"]

def load_fast_rows(path):
    data = open(path, "rb").read()
    off = struct.calcsize(HEADER_FMT)
    rows = []
    while off < len(data):
        rec_type = data[off]
        size = SIZES.get(rec_type)
        if size is None or off + size > len(data):
            break
        if rec_type == RECORD_FAST:
            (_, ms, a1, a2, b1, b2, hv, trq, rpm, busv, busi,
             flags) = struct.unpack_from(FAST_FMT, data, off)
            rows.append((ms, b1, b2, hv, trq / DIVISOR_TENTHS, rpm, busv / DIVISOR_TENTHS,
                         busi / DIVISOR_TENTHS, flags))
        off += size
    return rows

def print_fast_window(path, t0, t1, step=1):
    rows = load_fast_rows(path)
    print(f"\n--- {path} {t0}..{t1}s ---")
    print("t_s     bse1  bse2  hvraw  cmdNm   rpm   busV   busA  flags")
    last = None
    for i, r in enumerate(rows):
        t = r[0] / MS_PER_S
        if t0 <= t <= t1 and i % step == 0:
            fl = [n for b, n in enumerate(FLAGS) if r[8] & (1 << b)]
            line = (f"{t:7.2f} {r[1]:5d} {r[2]:5d} {r[3]:6d} {r[4]:6.1f} "
                    f"{r[5]:5d} {r[6]:6.1f} {r[7]:6.1f}  {','.join(fl)}")
            print(line)

# torque recheck >=1000 rpm across the third set's power runs
SET = "3rd set of runs"
for f in ["LOG0010.BIN", "LOG0011.BIN", "LOG0012.BIN"]:
    rows = load_fast_rows(f"{SET}/{f}")
    best = max(((NM_RPM_PER_KW * v * i / W_PER_KW / rpm, ms, v * i / W_PER_KW, rpm, trq)
                for ms, b1, b2, hv, trq, rpm, v, i, fl in rows if rpm >= MIN_RPM_FOR_TORQUE),
               default=None)
    if best:
        print(f"{f}: max calc torque (rpm>=1000): {best[0]:.1f} Nm "
              f"({best[2]:.1f} kW @ {best[3]} rpm, cmd {best[4]:.1f} Nm, t={best[1]/MS_PER_S:.1f}s)")

# context around real BSPD-low events found by analyze_runs in this set
print_fast_window(f"{SET}/LOG0008.BIN", 50.5, 56.5, 4)   # longest stationary BSPD-low window
print_fast_window(f"{SET}/LOG0011.BIN", 402.3, 402.9, 1) # brief BSPD-low at end of driving run
print_fast_window(f"{SET}/LOG0012.BIN", 1069.0, 1069.6, 1)
