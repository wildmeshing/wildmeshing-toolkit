#!/usr/bin/env python3
"""Where did each timed-out model die, how far did it get, and at what energy."""
import os
import re
import collections
import statistics as st

ROOT = os.environ.get("TRIWILD_ROOT", "/u/3/daniele/triwild-sweep")
R = os.environ.get("TRIWILD_OUT", os.path.join(ROOT, "runs/full"))
FD = os.path.join(R, "failure")
DATA = os.path.join(ROOT, "data")

# Milestones in the order triwild logs them. The last one present is how far it got.
PHASES = [
    ("Read edge mesh", "read input"),
    ("input simplification:", "simplified"),
    ("skip simplification", "simplified"),
    ("2D arrangement:", "arrangement done"),
    ("Envelope sanity check", "init_mesh done"),
    ("========it pre========", "pre-collapse"),
    ("========it 0========", "optimization"),
    ("========it post========", "post-collapse"),
    ("flood fill parts", "winding number"),
    ("Hausdorff distance", "hausdorff check"),
    ("final max energy", "writing output"),
]

rows = []
for m in sorted(os.listdir(FD)):
    d = os.path.join(FD, m)
    log = ""
    for f in ("out.log", "console.log"):
        p = os.path.join(d, f)
        if os.path.exists(p):
            log += open(p, errors="ignore").read()
    try:
        size = os.path.getsize(os.path.join(DATA, m + ".obj"))
    except OSError:
        size = 0

    phase = "before reading"
    for needle, name in PHASES:
        if needle in log:
            phase = name

    its = len(re.findall(r"========it (\d+)========", log))
    en = re.findall(r"max energy ([0-9.e+]+) \| stop", log)
    first = float(en[0]) if en else None
    last = float(en[-1]) if en else None
    nt = re.findall(r"#V = \d+, #T = (\d+)", log)
    ne = re.search(r"Read edge mesh \S+: #V = (\d+), #E = (\d+)", log)
    simp = re.search(r"#E (\d+) -> (\d+)", log)
    arr = re.search(r"2D arrangement: #V = (\d+), #F = (\d+)", log)
    rows.append(
        dict(m=m, size=size, phase=phase, its=its, first=first, last=last,
             nt=int(nt[-1]) if nt else None,
             in_e=int(ne.group(2)) if ne else None,
             simp_e=int(simp.group(2)) if simp else None,
             arr_f=int(arr.group(2)) if arr else None))

print("%d failures\n" % len(rows))

# A sweep that finished cleanly has none, which is the good case, not an error.
if not rows:
    print("nothing to report: %s holds no failed models." % FD)
    raise SystemExit(0)

print("=== how far each got ===")
order = [n for _, n in PHASES]
counts = collections.Counter(r["phase"] for r in rows)
for name in ["before reading"] + list(dict.fromkeys(order)):
    if counts.get(name):
        print("  %5d  %s" % (counts[name], name))

print("\n=== iterations of the optimization loop reached ===")
its = [r["its"] for r in rows]
buckets = [(0, 0), (1, 1), (2, 5), (6, 20), (21, 200)]
for lo, hi in buckets:
    c = sum(1 for i in its if lo <= i <= hi)
    if c:
        print("  %5d models: %s iterations" % (c, lo if lo == hi else "%d-%d" % (lo, hi)))
print("  median %d, max %d" % (st.median(its), max(its)))

print("\n=== input size ===")
sz = sorted(r["size"] for r in rows)
print("  median %.0f MB   min %.1f MB   max %.0f MB" % (
    sz[len(sz) // 2] / 1e6, min(sz) / 1e6, max(sz) / 1e6))

print("\n=== energy, for the ones that ran at least one iteration ===")
run = [r for r in rows if r["its"] > 0 and r["last"] is not None]
print("  %d of %d models" % (len(run), len(rows)))
if run:
    lasts = sorted(r["last"] for r in run)
    print("  last max energy: median %.4g   min %.4g   max %.4g   (target 20)" % (
        lasts[len(lasts) // 2], min(lasts), max(lasts)))
    print("  still above target: %d of %d" % (sum(1 for v in lasts if v > 20), len(lasts)))
    conv = sum(1 for r in run if r["first"] and r["last"] and r["last"] < r["first"])
    print("  energy was still decreasing when killed: %d of %d" % (conv, len(run)))

print("\n=== furthest into the optimization ===")
hdr = "  %-9s %6s %6s %9s %12s %12s  %s"
print(hdr % ("model", "MB", "iters", "#T", "first maxE", "last maxE", "phase"))
for r in sorted(rows, key=lambda r: -r["its"])[:12]:
    print(hdr % (r["m"], "%.0f" % (r["size"] / 1e6), r["its"],
                 r["nt"] if r["nt"] else "-",
                 "%.4g" % r["first"] if r["first"] else "-",
                 "%.4g" % r["last"] if r["last"] else "-", r["phase"]))

print("\n=== smallest inputs that still timed out ===")
hdr2 = "  %-9s %6s %6s %10s %10s %9s  %s"
print(hdr2 % ("model", "MB", "iters", "input #E", "simp #E", "arr #F", "phase"))
for r in sorted(rows, key=lambda r: r["size"])[:10]:
    print(hdr2 % (r["m"], "%.1f" % (r["size"] / 1e6), r["its"],
                  r["in_e"] if r["in_e"] else "-",
                  r["simp_e"] if r["simp_e"] else "-",
                  r["arr_f"] if r["arr_f"] else "-", r["phase"]))
