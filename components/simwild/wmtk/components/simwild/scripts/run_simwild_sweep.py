#!/usr/bin/env python3
"""
Batch-run simwild over the Thingi10K dataset -- kirby.cs.nyu.edu edition.

Adapted from tetwild's run_tetwild_sweep.py: same contract, environment layout and
output shape (only the TETWILD_* variables become SIMWILD_*), so a simwild sweep and
a tetwild sweep can run side by side on the same machine without either one's settings
leaking into the other. Run it with no arguments to start (or resume) the sweep; it
skips every model already in success/ or failure/. Stop it cleanly with:

    run_simwild_sweep.py stop

which signals the running sweep to abandon whatever is in flight (those models are
left unpublished, so a later resume reprocesses them -- no spurious failures), drain,
write the report, and exit.

Each model is a single closed surface mesh, fed through simwild with
tag_from_winding_number left at its default (true): the arrangement's one cell is
tagged inside/outside by winding number, so no curve network or pre-tagged .msh is
needed -- the same single-input shape tetwild's Thingi10K sweep uses.

What changed from tetwild's version:
  * application is "simwild", not "tetwild";
  * simwild has no `filter` knob (that is tetwild's post-insertion outside-removal
    pass; simwild's single-input case is tagged by winding number instead), so it is
    dropped from PARAMS rather than pinned;
  * `eps_rel` is pinned at simwild's own spec default (2e-3), not tetwild's (1e-3) --
    the two applications do not share a default;
  * paths point at /u/3/daniele/simwild-thingi10k-sweep, a separate root from
    tetwild's sweep so the two never share a success/failure namespace;
  * SIMWILD_SAMPLE picks *which* models a truncated run gets -- 'smallest' is the
    original behaviour, 'spread' covers the whole size range, which is what you want
    from a 100-model trial;
  * the report tells OOM kills apart from other signals.

Per model:
  * write a simwild JSON (eps_rel 2e-3, single-input winding-number tagging);
  * run wmtk_app in a scratch dir under the output volume, capped in time and memory;
  * exit 0                    -> move the run into  <OUT>/success/<id>/
    nonzero / timeout / OOM   -> move the run into  <OUT>/failure/<id>/  (with a reason)

Configuration -- environment variables:
    SIMWILD_ROOT          sweep root: build/ data/ runs/ live here
    SIMWILD_OUT           output directory            (default runs/full)
    SIMWILD_PARALLEL      models to run concurrently  (default 8)
    SIMWILD_THREADS       threads per model           (default 8)
    SIMWILD_JOB_TIMEOUT   per-model seconds           (default 10800 = 3h)
    SIMWILD_MEM_GB        per-model memory cap, GB    (default 128, 0 disables)
    SIMWILD_LIMIT         process at most N new models (default 0 = all)
    SIMWILD_SAMPLE        name | smallest | spread | random  (default smallest)
    SIMWILD_SEED          seed for SIMWILD_SAMPLE=random  (default 0)
    SIMWILD_REPORT_ONLY   if set, only regenerate the report and exit

Memory note: the cap is PER MODEL, not a budget for the sweep. 8 x 128G is more than
kirby has, so the cap is a runaway-killer, not an admission control -- it stops one
pathological mesh from swapping the box, and does nothing in the normal case. Lower
SIMWILD_MEM_GB if the sweep has to coexist with someone else's job.
"""

import csv
import datetime
import json
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor
from concurrent.futures import wait as futures_wait
from pathlib import Path

# --------------------------------------------------------------------------- #
# Layout -- rooted at SIMWILD_ROOT, defaulting to the kirby path
# --------------------------------------------------------------------------- #
SWEEP_ROOT = Path(os.environ.get("SIMWILD_ROOT", "/u/3/daniele/simwild-thingi10k-sweep"))
WMTK_APP = SWEEP_ROOT / "build/app/wmtk_app"
DATASET_DIR = SWEEP_ROOT / "data"
OUT_DIR = Path(os.environ.get("SIMWILD_OUT", str(SWEEP_ROOT / "runs/full")))

# Thingi10K's 10,000 raw meshes are mostly STL, plus a handful of obj/ply/off;
# igl::read_triangle_mesh reads all four directly. Every filename stem is unique
# across the extensions, so the stem is a safe per-model id.
MESH_EXTENSIONS = ("*.stl", "*.obj", "*.ply", "*.off")

PARALLEL = max(1, int(os.environ.get("SIMWILD_PARALLEL", "8")))
THREADS = max(1, int(os.environ.get("SIMWILD_THREADS", "8")))
JOB_TIMEOUT = int(os.environ.get("SIMWILD_JOB_TIMEOUT", "10800"))  # seconds, 3h
MEM_GB = int(os.environ.get("SIMWILD_MEM_GB", "128"))  # 0 = no cap
LIMIT = int(os.environ.get("SIMWILD_LIMIT", "0"))  # 0 = no limit
SAMPLE = os.environ.get("SIMWILD_SAMPLE", "smallest")
SEED = int(os.environ.get("SIMWILD_SEED", "0"))
REPORT_ONLY = bool(os.environ.get("SIMWILD_REPORT_ONLY"))

# simwild parameters requested for this sweep. Everything not named here comes from
# the spec defaults of whatever branch build/ was built from. eps_rel below happens
# to equal the current spec default; it is pinned so the sweep keeps meaning the same
# thing if that default ever moves. preserve_topology is the one deliberate departure
# from the defaults, so this sweep is not a pure defaults run -- results from it are
# not comparable with a run made before it was pinned. Unlike tetwild, there is no
# `filter` knob to pin: a single-input run is tagged inside/outside by winding number
# (tag_from_winding_number defaults to true), not by a post-insertion outside-removal
# pass.
PARAMS = {
    "application": "simwild",
    "eps_rel": 2e-3,
    "num_threads": THREADS,
    # Deliberately *not* the spec default (which is true). Requested for this sweep:
    # with preserve_topology off, simplification runs pre-insertion and is allowed to
    # change input topology, which is the configuration under test here.
    "preserve_topology": False,
    # The .msh is the real output and the .vtu is a visualization dump that _prune
    # deletes immediately afterwards, so writing it costs time and disk on every model
    # for nothing. The 2D sweep has always forced this off.
    "write_vtu": False,
}

# Which of a run's output files to keep. The .vtu files are large visualization
# dumps; the .msh is the actual tet mesh. Flip KEEP_GLOBS to ["*"] to keep everything.
#
# Text only on this machine: the geometry does not fit. Measured on the 1000-model
# tetwild run in wmtk-t10k/tetwild_outputs, the .msh averages 19 MB/model -- ~200 GB
# over the full corpus, against 164 GB free on the largest volume here. Nothing in
# the report reads the geometry: it is built from status.txt, report.json and out.log
# alone, so dropping *.msh and *.obj costs no statistic. Restore them on a box with
# the space if the meshes themselves are wanted.
KEEP_GLOBS = ["*.log", "*.json", "status.txt"]

SUCCESS_DIR = OUT_DIR / "success"
FAILURE_DIR = OUT_DIR / "failure"
WORK_DIR = OUT_DIR / ".work"  # scratch, same filesystem as OUT so publish is atomic
REPORT_DIR = OUT_DIR / "report"
PIDFILE = OUT_DIR / "sweep.pid"

# --------------------------------------------------------------------------- #
# Stop handling.
#
# A stop must be clean under parallelism: kill every in-flight wmtk_app, and have
# each worker ABANDON its model -- leave it unpublished so a resume reprocesses it --
# rather than record the kill as a failure. _STOP is the flag; _children tracks the
# live subprocesses so the signal handler can kill them all.
# --------------------------------------------------------------------------- #
_STOP = threading.Event()
_children = {}  # model_id -> subprocess.Popen
_children_lock = threading.Lock()


def _kill(popen):
    if popen and popen.poll() is None:
        try:
            os.killpg(os.getpgid(popen.pid), signal.SIGKILL)  # whole thread group
        except (ProcessLookupError, PermissionError):
            pass


def _handle_signal(signum, _frame):
    if not _STOP.is_set():
        _STOP.set()
        print(f"\n[signal] {signal.Signals(signum).name} received; abandoning in-flight "
              f"models (they resume later), draining, and writing the report.", flush=True)
    with _children_lock:
        for popen in _children.values():
            _kill(popen)


signal.signal(signal.SIGINT, _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)


# --------------------------------------------------------------------------- #
# Memory cap.
#
# cgroup v2 via a systemd transient scope, rather than RLIMIT_AS: the toolkit's
# dependencies (TBB, geogram) reserve large virtual ranges they never fault in, so an
# address-space limit fires on models that are nowhere near the memory it is meant to
# bound. MemoryMax is real usage. MemorySwapMax=0 keeps a bloated model from dragging
# the whole machine into swap instead of dying.
#
# The scope is a child in our own process group, so the existing killpg path still
# terminates it; the cap only changes how the kernel treats its memory.
# --------------------------------------------------------------------------- #
def _mem_wrapper(model_id):
    if MEM_GB <= 0:
        return []
    return [
        "systemd-run", "--user", "--scope", "--quiet",
        f"--unit=sw-{model_id}-{os.getpid()}",
        "-p", f"MemoryMax={MEM_GB}G",
        "-p", "MemorySwapMax=0",
        "--",
    ]


def _memory_capped_available():
    """Probe once, so a broken systemd user manager fails loudly at startup."""
    if MEM_GB <= 0:
        return True
    try:
        r = subprocess.run(
            ["systemd-run", "--user", "--scope", "--quiet",
             "-p", f"MemoryMax={MEM_GB}G", "--", "/bin/true"],
            capture_output=True, timeout=30)
        return r.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


# --------------------------------------------------------------------------- #
# Running one model
# --------------------------------------------------------------------------- #
def already_done(model_id):
    return (SUCCESS_DIR / model_id).exists() or (FAILURE_DIR / model_id).exists()


def run_one(mesh_path):
    """Run simwild on one mesh. Returns 'success' / 'failure' / 'stopped'."""
    model_id = mesh_path.stem
    if _STOP.is_set():
        return "stopped"  # never started

    work = WORK_DIR / model_id
    if work.exists():
        shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)

    cfg = dict(PARAMS)
    cfg["input"] = [str(mesh_path)]
    cfg["output"] = "out"
    cfg["log_file"] = "out.log"
    cfg["report"] = "report.json"
    config_path = (work / "config.json").resolve()
    config_path.write_text(json.dumps(cfg, indent=2))

    print(f"  .. start {model_id} ({mesh_path.stat().st_size // 1024} KB)", flush=True)
    reason = ""
    started = time.time()
    with open(work / "console.log", "wb") as console:
        # start_new_session so the whole thread group can be killed as a unit.
        popen = subprocess.Popen(
            # Absolute config path, not "config.json": app/main.cpp sets input_dir to
            # the config's parent_path(), and wmtk::utils::resolve_path calls
            # fs::absolute() on it. libstdc++ throws on the empty path, so a relative
            # config in the cwd aborts every model on Linux (macOS's libc++ does not).
            _mem_wrapper(model_id) + [str(WMTK_APP), "-j", str(config_path)],
            cwd=str(work),
            stdout=console,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        with _children_lock:
            _children[model_id] = popen
        try:
            if _STOP.is_set():  # stop raced in just after Popen; kill immediately
                _kill(popen)
            code = popen.wait(timeout=JOB_TIMEOUT)
        except subprocess.TimeoutExpired:
            _kill(popen)
            popen.wait()
            code = None
            reason = f"timeout (> {JOB_TIMEOUT}s)"
        finally:
            with _children_lock:
                _children.pop(model_id, None)
    elapsed = time.time() - started

    # A stop in progress abandons whatever is in flight: drop the scratch, publish
    # nothing, let the resume reprocess it. (A model genuinely crashing at stop time
    # simply re-runs and is recorded on resume -- no harm.)
    if _STOP.is_set():
        shutil.rmtree(work, ignore_errors=True)
        print(f"  .. abandoned (stop): {model_id}", flush=True)
        return "stopped"

    if code == 0:
        status = "success"
    else:
        status = "failure"
        if not reason:
            reason = _describe_exit(code, work)

    # status.txt: machine-parseable, one 'key: value' per line.
    (work / "status.txt").write_text(
        f"model: {model_id}\n"
        f"input: {mesh_path}\n"
        f"status: {status}\n"
        f"reason: {reason}\n"
        f"exit_code: {code}\n"
        f"wall_seconds: {elapsed:.3f}\n"
    )

    # Drop the files we do not keep (large .vtu), then publish atomically.
    _prune(work)
    dest = (SUCCESS_DIR if status == "success" else FAILURE_DIR) / model_id
    if dest.exists():
        shutil.rmtree(dest, ignore_errors=True)
    os.replace(work, dest)  # atomic rename within the same filesystem

    print(f"  -> {status}: {model_id}  ({elapsed:.1f}s"
          + (f", {reason}" if reason else "") + ")", flush=True)
    return status


def _describe_exit(code, work):
    """Human-readable reason for a nonzero exit, distinguishing the OOM kill.

    A cgroup OOM kill arrives as SIGKILL, which is also what a manual kill looks
    like -- but we only kill on stop or timeout, and both are handled before this
    is reached, so an unexplained SIGKILL under a cap is the OOM killer. systemd
    says so on the scope's stderr when it can, so prefer that when it is there.
    """
    console = work / "console.log"
    if console.exists():
        tail = console.read_text(errors="ignore")[-4000:].lower()
        if "oom" in tail or "out of memory" in tail:
            return f"OOM (MemoryMax={MEM_GB}G)"
    if code is not None and code < 0:
        signame = signal.Signals(-code).name
        if -code == signal.SIGKILL and MEM_GB > 0:
            return f"killed by SIGKILL, no timeout -- OOM (MemoryMax={MEM_GB}G)"
        return f"killed by signal {-code} ({signame})"
    if code == 137 and MEM_GB > 0:
        return f"exit 137 (SIGKILL) -- OOM (MemoryMax={MEM_GB}G)"
    return f"nonzero exit ({code})"


def _prune(work):
    keep = set()
    for pat in KEEP_GLOBS:
        keep.update(work.glob(pat))
    for f in work.iterdir():
        if f.is_file() and f not in keep:
            f.unlink()


# --------------------------------------------------------------------------- #
# Report generation
# --------------------------------------------------------------------------- #
# "========it N========" is logged by the shared TetOptimizerMesh/TriOptimizerMesh
# main loop that simwild runs on top of, so this pattern is unchanged from tetwild.
IT_RE = re.compile(r"========it (\d+)========")


def _read_status(d):
    info = {}
    st = d / "status.txt"
    if st.exists():
        for line in st.read_text().splitlines():
            if ":" in line:
                k, _, v = line.partition(":")
                info[k.strip()] = v.strip()
    return info


def _parse_success(d):
    """time, iterations, energies for one successful model."""
    row = {"id": d.name, "status": "success"}
    rep = d / "report.json"
    if rep.exists():
        try:
            j = json.loads(rep.read_text())
            row["time"] = j.get("time")
            row["max_energy"] = j.get("max_energy")
            row["avg_energy"] = j.get("avg_energy")
            row["verts"] = j.get("#v")
            row["tets"] = j.get("#t")
        except json.JSONDecodeError:
            pass
    log = d / "out.log"
    if log.exists():
        text = log.read_text(errors="ignore")
        iters = IT_RE.findall(text)
        # count numeric iteration markers ("it pre"/"it post" are not numeric)
        row["iterations"] = len(iters)
        if row.get("time") is None:
            m = re.search(r"total time (\S+)s", text)
            if m:
                row["time"] = float(m.group(1))
    st = _read_status(d)
    if row.get("time") is None and st.get("wall_seconds"):
        row["time"] = float(st["wall_seconds"])
    return row


def _histogram(title, values, edges, fmt="{:g}"):
    """ASCII histogram. edges is a list of bucket boundaries; last bucket is open."""
    if not values:
        return f"{title}: (no data)\n"
    counts = [0] * (len(edges) + 1)
    for v in values:
        placed = False
        for i, e in enumerate(edges):
            if v < e:
                counts[i] += 1
                placed = True
                break
        if not placed:
            counts[-1] += 1
    labels = []
    for i in range(len(edges) + 1):
        if i == 0:
            labels.append(f"< {fmt.format(edges[0])}")
        elif i == len(edges):
            labels.append(f">= {fmt.format(edges[-1])}")
        else:
            labels.append(f"{fmt.format(edges[i-1])} - {fmt.format(edges[i])}")
    width = max(len(x) for x in labels)
    peak = max(counts) or 1
    lines = [title]
    for lab, c in zip(labels, counts):
        bar = "#" * int(round(50 * c / peak))
        lines.append(f"  {lab:>{width}} | {bar} {c}")
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- #
# HTML report
# --------------------------------------------------------------------------- #

_CSS = """
:root {
  color-scheme: light;
  --surface-1: #fcfcfb; --plane: #f9f9f7;
  --ink-1: #0b0b0b; --ink-2: #52514e; --ink-muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7;
  --series-1: #2a78d6; --series-2: #eb6834; --series-3: #1baf7a;
  --good: #0ca30c; --critical: #d03b3b; --warning: #ec835a;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --surface-1: #1a1a19; --plane: #0d0d0d;
    --ink-1: #ffffff; --ink-2: #c3c2b7; --ink-muted: #898781;
    --grid: #2c2c2a; --axis: #383835;
    --series-1: #3987e5; --series-2: #d95926; --series-3: #199e70;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --surface-1: #1a1a19; --plane: #0d0d0d;
  --ink-1: #ffffff; --ink-2: #c3c2b7; --ink-muted: #898781;
  --grid: #2c2c2a; --axis: #383835;
  --series-1: #3987e5; --series-2: #d95926; --series-3: #199e70;
}

* { box-sizing: border-box; }
body {
  margin: 0; padding: 2rem 1.25rem 4rem;
  background: var(--plane); color: var(--ink-1);
  font: 15px/1.55 ui-sans-serif, -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
}
.wrap { max-width: 1080px; margin: 0 auto; }
h1 { font-size: 1.65rem; letter-spacing: -0.02em; margin: 0 0 0.2rem; }
h2 {
  font-size: 1.02rem; letter-spacing: -0.01em; margin: 2.4rem 0 0.85rem;
  padding-bottom: 0.45rem; border-bottom: 1px solid var(--grid);
}
h3 { margin: 0 0 0.7rem; font-size: 0.9rem; }
.sub { color: var(--ink-2); font-size: 0.9rem; margin: 0 0 1.6rem; }
.sub code { font-size: 0.85em; color: var(--ink-muted); }
.note { color: var(--ink-2); font-size: 0.87rem; margin: 0.55rem 0 0; max-width: 74ch; }
.banner {
  background: var(--surface-1); border: 1px solid var(--grid);
  border-left: 3px solid var(--warning); border-radius: 8px;
  padding: 0.7rem 0.9rem; margin: 0 0 1.3rem; font-size: 0.88rem; color: var(--ink-2);
}

.stats { display: flex; flex-wrap: wrap; gap: 0.6rem; margin: 0 0 0.5rem; }
.stat {
  flex: 1 1 150px; background: var(--surface-1);
  border: 1px solid var(--grid); border-radius: 10px; padding: 0.85rem 0.95rem;
}
.stat .k {
  font-size: 0.72rem; text-transform: uppercase; letter-spacing: 0.055em;
  color: var(--ink-muted); margin-bottom: 0.3rem;
}
.stat .v { font-size: 1.5rem; font-weight: 620; letter-spacing: -0.02em;
           font-variant-numeric: tabular-nums; }
.stat .u { font-size: 0.79rem; color: var(--ink-2); margin-top: 0.15rem; }
.stat.ok .v { color: var(--good); }
.stat.bad .v { color: var(--critical); }

.card {
  background: var(--surface-1); border: 1px solid var(--grid);
  border-radius: 10px; padding: 1.05rem 1.15rem 1.15rem;
}
.chart { display: grid; grid-template-columns: max-content 1fr; gap: 2px 0.7rem; }
.chart .lab {
  font-size: 0.79rem; color: var(--ink-2); text-align: right;
  font-variant-numeric: tabular-nums; align-self: center; white-space: nowrap;
}
.track {
  position: relative; height: 21px; display: flex; align-items: center;
  border-left: 1px solid var(--axis);
}
.bar { height: 15px; border-radius: 0 4px 4px 0; background: var(--series-1);
       min-width: 2px; transition: filter .12s; }
.track:hover .bar { filter: brightness(1.12); }
.bar.zero { background: none; border: 1px dashed var(--axis); height: 13px; min-width: 12px; }
.bar.alt { background: var(--series-2); }
.cnt { font-size: 0.76rem; color: var(--ink-2); margin-left: 0.45rem;
       font-variant-numeric: tabular-nums; white-space: nowrap; }
.grid2 { display: grid; grid-template-columns: repeat(auto-fit, minmax(430px, 1fr)); gap: 0.9rem; }

.tblwrap { overflow-x: auto; background: var(--surface-1);
           border: 1px solid var(--grid); border-radius: 10px; }
table { border-collapse: collapse; width: 100%; font-size: 0.86rem; }
th, td { padding: 0.42rem 0.75rem; text-align: right; white-space: nowrap; }
th { font-size: 0.71rem; text-transform: uppercase; letter-spacing: 0.05em;
     color: var(--ink-muted); font-weight: 600; border-bottom: 1px solid var(--grid);
     position: sticky; top: 0; background: var(--surface-1); }
td { border-bottom: 1px solid var(--grid); font-variant-numeric: tabular-nums; }
tr:last-child td { border-bottom: none; }
th:first-child, td:first-child { text-align: left; }
td.reason { text-align: left; white-space: normal; color: var(--ink-2); }
details { margin-top: 0.7rem; }
summary { cursor: pointer; color: var(--ink-2); font-size: 0.85rem;
          padding: 0.3rem 0; user-select: none; }
summary:hover { color: var(--ink-1); }

#tip {
  position: fixed; pointer-events: none; opacity: 0; transition: opacity .1s;
  background: var(--ink-1); color: var(--surface-1);
  padding: 0.35rem 0.55rem; border-radius: 6px; font-size: 0.78rem;
  font-variant-numeric: tabular-nums; z-index: 50; max-width: 280px;
}
footer { color: var(--ink-muted); font-size: 0.8rem; margin-top: 2.5rem;
         padding-top: 0.9rem; border-top: 1px solid var(--grid); }
"""

_JS = """
(function () {
  var tip = document.getElementById('tip');
  document.querySelectorAll('[data-tip]').forEach(function (el) {
    el.addEventListener('mousemove', function (e) {
      tip.textContent = el.getAttribute('data-tip');
      tip.style.opacity = '1';
      var x = e.clientX + 14, y = e.clientY + 16;
      var w = tip.offsetWidth, h = tip.offsetHeight;
      if (x + w > window.innerWidth - 8) x = e.clientX - w - 14;
      if (y + h > window.innerHeight - 8) y = e.clientY - h - 16;
      tip.style.left = x + 'px'; tip.style.top = y + 'px';
    });
    el.addEventListener('mouseleave', function () { tip.style.opacity = '0'; });
  });
})();
"""


def _esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def _fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float):
        if v != v:
            return "-"
        if v >= 1e5 or (v and abs(v) < 1e-3):
            return "{:.3g}".format(v)
        return "{:.{}f}".format(v, nd)
    return str(v)


def _bucket(values, edges):
    counts = [0] * (len(edges) + 1)
    for v in values:
        for i, e in enumerate(edges):
            if v < e:
                counts[i] += 1
                break
        else:
            counts[-1] += 1
    labels = []
    for i in range(len(edges) + 1):
        if i == 0:
            labels.append("< {:g}".format(edges[0]))
        elif i == len(edges):
            labels.append(">= {:g}".format(edges[-1]))
        else:
            labels.append("{:g} - {:g}".format(edges[i - 1], edges[i]))
    return labels, counts


def _bars(labels, counts, unit, alt=False):
    peak = max(counts) if counts else 0
    total = sum(counts) or 1
    out = ['<div class="chart">']
    for lab, c in zip(labels, counts):
        cls = "bar" + (" zero" if c == 0 else (" alt" if alt else ""))
        w = 100.0 * c / peak if peak else 0.0
        tip = "{} {}: {} models ({:.1f}%)".format(lab, unit, c, 100.0 * c / total)
        out.append('<div class="lab">{}</div>'.format(_esc(lab)))
        out.append('<div class="track" data-tip="{}">'
                   '<div class="{}" style="width:{:.2f}%"></div>'
                   '<span class="cnt">{}</span></div>'.format(
                       _esc(tip), cls, w if c else 0.0, "{:,}".format(c)))
    out.append("</div>")
    return "".join(out)


def _html_hist(values, edges, unit):
    if not values:
        return '<p class="note">(no data)</p>'
    labels, counts = _bucket(values, edges)
    return _bars(labels, counts, unit)


def _stat(k, v, u="", cls=""):
    return ('<div class="stat {}"><div class="k">{}</div><div class="v">{}</div>'
            '<div class="u">{}</div></div>').format(cls, _esc(k), _esc(v), _esc(u))


def _table(headers, rows, reason_col=None, empty="(none)"):
    if not rows:
        return '<p class="note">{}</p>'.format(_esc(empty))
    h = "".join("<th>{}</th>".format(_esc(x)) for x in headers)
    body = []
    for r in rows:
        tds = []
        for i, c in enumerate(r):
            cls = ' class="reason"' if reason_col is not None and i == reason_col else ""
            tds.append("<td{}>{}</td>".format(cls, _esc(c)))
        body.append("<tr>" + "".join(tds) + "</tr>")
    return ('<div class="tblwrap"><table><thead><tr>{}</tr></thead>'
            "<tbody>{}</tbody></table></div>").format(h, "".join(body))


def generate_html_report(rows, stats, report_dir, out_dir, dataset_dir=None,
                         mesh_extensions=()):
    """Write report/index.html. `rows` are the same dicts the CSV is built from."""
    ok = [r for r in rows if r.get("status") == "success"]
    bad = [r for r in rows if r.get("status") != "success"]
    total = len(rows)

    def col(key, src=None):
        return [r[key] for r in (src if src is not None else ok)
                if isinstance(r.get(key), (int, float))]

    times, iters, energies = col("time"), col("iterations"), col("max_energy")
    tets = col("tets")

    def med(xs):
        return sorted(xs)[len(xs) // 2] if xs else None

    rate = (100.0 * len(ok) / total) if total else 0.0

    # Input size on disk, keyed by model id -- the 3D sweep records no input vertex
    # count, and file size is the honest proxy anyway since the whole file is read up
    # front. Missing files just drop out of the scaling table.
    sizes = {}
    if dataset_dir is not None:
        import os as _os
        import glob as _glob
        for pat in (mesh_extensions or ("*",)):
            for p in _glob.glob(_os.path.join(str(dataset_dir), pat)):
                stem = _os.path.basename(p).rsplit(".", 1)[0]
                try:
                    sizes[stem] = _os.path.getsize(p)
                except OSError:
                    pass

    h = []
    h.append("<!doctype html><html lang='en'><head><meta charset='utf-8'>")
    h.append("<meta name='viewport' content='width=device-width,initial-scale=1'>")
    h.append("<title>simwild Thingi10K sweep report</title>")
    h.append("<style>{}</style></head><body><div id='tip'></div><div class='wrap'>".format(_CSS))

    h.append("<h1>simwild &mdash; Thingi10K sweep</h1>")
    h.append("<p class='sub'>{} models &middot; <code>{}</code> &middot; generated {}</p>".format(
        "{:,}".format(total), _esc(out_dir), _esc(stats.get("generated", ""))))

    if stats.get("banner"):
        h.append("<div class='banner'>{}</div>".format(_esc(stats["banner"])))

    h.append("<div class='stats'>")
    h.append(_stat("Processed", "{:,}".format(total)))
    h.append(_stat("Success rate", "{:.2f}%".format(rate),
                   "{:,} succeeded".format(len(ok)), "ok" if not bad else ""))
    h.append(_stat("Failures", "{:,}".format(len(bad)), "of {:,}".format(total),
                   "bad" if bad else "ok"))
    h.append(_stat("Median iterations", _fmt(med(iters), 0), "per model"))
    h.append("</div><div class='stats'>")
    h.append(_stat("Median run time", _fmt(med(times), 1), "seconds"))
    h.append(_stat("Max run time", _fmt(max(times), 0) if times else "-", "seconds"))
    h.append(_stat("Median max energy", _fmt(med(energies), 2), "AMIPS3D floor is 3"))
    h.append(_stat("Median #T", "{:,}".format(int(med(tets))) if tets else "-", "tets out"))
    h.append("</div>")

    h.append("<h2>Distributions</h2><div class='grid2'>")
    h.append("<div class='card'><h3>Run time (seconds)</h3>{}</div>".format(
        _html_hist(times, [1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600], "s")))
    h.append("<div class='card'><h3>Iterations</h3>{}</div>".format(
        _html_hist(iters, [1, 2, 3, 5, 8, 12, 20, 40, 80], "iterations")))
    h.append("<div class='card'><h3>Final max energy "
             "<span style='color:var(--ink-muted);font-weight:400'>(AMIPS3D floor is 3)"
             "</span></h3>{}</div>".format(
                 _html_hist(energies, [5, 10, 20, 50, 100, 101], "energy")))
    h.append("<div class='card'><h3>Output size (#T)</h3>{}</div>".format(
        _html_hist(tets, [1e3, 1e4, 5e4, 1e5, 5e5, 1e6], "tets")))
    h.append("</div>")

    # ---- scaling ----
    sized = [r for r in ok if r["id"] in sizes and isinstance(r.get("time"), (int, float))]
    if sized:
        h.append("<h2>Run time vs input size</h2>")
        edges = [1e5, 1e6, 5e6, 2e7, 1e8]
        buckets = [[] for _ in range(len(edges) + 1)]
        for r in sized:
            s = sizes[r["id"]]
            for i, e in enumerate(edges):
                if s < e:
                    buckets[i].append(r)
                    break
            else:
                buckets[-1].append(r)
        labels, _ = _bucket([sizes[r["id"]] for r in sized], edges)
        labels = [x.replace("1e+05", "100 KB").replace("1e+06", "1 MB")
                   .replace("5e+06", "5 MB").replace("2e+07", "20 MB")
                   .replace("1e+08", "100 MB") for x in labels]
        trows = []
        for lab, b in zip(labels, buckets):
            if not b:
                continue
            ts = sorted(r["time"] for r in b)
            p95 = ts[min(len(ts) - 1, int(0.95 * len(ts)))]
            its = [r["iterations"] for r in b if isinstance(r.get("iterations"), int)]
            nt = [r["tets"] for r in b if isinstance(r.get("tets"), int)]
            trows.append([lab, "{:,}".format(len(b)), _fmt(ts[len(ts) // 2], 1),
                          _fmt(p95, 1), _fmt(max(ts), 1),
                          _fmt(sum(its) / len(its), 1) if its else "-",
                          "{:,}".format(int(sorted(nt)[len(nt) // 2])) if nt else "-"])
        h.append(_table(["input file", "models", "median s", "p95 s", "max s",
                         "mean iters", "median #T"], trows))

    # ---- failures ----
    h.append("<h2>Failures</h2>")
    if bad:
        by_reason = {}
        for r in bad:
            key = (r.get("reason") or "unknown")
            key = ("OOM" if "oom" in key.lower() else
                   "timeout" if "timeout" in key.lower() else
                   "unreadable input / abort" if "signal 6" in key.lower() else key)
            by_reason.setdefault(key, []).append(r)
        ks = sorted(by_reason, key=lambda k: -len(by_reason[k]))
        h.append("<div class='card'><h3>By reason</h3>{}</div>".format(
            _bars(ks, [len(by_reason[k]) for k in ks], "", alt=True)))
        h.append(_table(["id", "reason", "wall s"],
                        [[r["id"], r.get("reason") or "unknown", _fmt(r.get("time"), 1)]
                         for r in sorted(bad, key=lambda r: str(r["id"]))],
                        reason_col=1))
        h.append("<p class='note'>A model whose input cannot be read aborts with a "
                 "diagnostic and reaches the harness as SIGABRT, so those rows are data "
                 "defects in the corpus rather than meshing failures. OOM rows are killed "
                 "at the per-model MemoryMax, not by the timeout.</p>")
    else:
        h.append("<div class='stats'>{}</div>".format(
            _stat("Failures", "0", "every model produced an output", "ok")))

    # ---- slowest ----
    slow = sorted([r for r in ok if isinstance(r.get("time"), (int, float))],
                  key=lambda r: -r["time"])[:25]
    if slow:
        h.append("<h2>Slowest models</h2>")
        h.append(_table(
            ["id", "time s", "iters", "max energy", "#V out", "#T out", "input"],
            [[r["id"], _fmt(r.get("time"), 1), r.get("iterations", "-"),
              _fmt(r.get("max_energy"), 3),
              "{:,}".format(r["verts"]) if r.get("verts") else "-",
              "{:,}".format(r["tets"]) if r.get("tets") else "-",
              ("%.1f MB" % (sizes[r["id"]] / 1048576)) if r["id"] in sizes else "-"]
             for r in slow]))

    h.append("<details><summary>Full per-model data</summary>"
             "<p class='note'>{:,} rows in <code>{}</code> &mdash; one line per model with "
             "time, iterations, energies, element counts and the failure reason."
             "</p></details>".format(total, _esc(str(report_dir / "results.csv"))))

    h.append("<footer>simwild / Thingi10K &middot; {}</footer>".format(
        _esc(stats.get("generated", ""))))
    h.append("</div><script>{}</script></body></html>".format(_JS))

    path = report_dir / "index.html"
    path.write_text("".join(h))
    return path


def generate_report():
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    successes = sorted(d for d in SUCCESS_DIR.glob("*") if d.is_dir()) \
        if SUCCESS_DIR.exists() else []
    failures = sorted(d for d in FAILURE_DIR.glob("*") if d.is_dir()) \
        if FAILURE_DIR.exists() else []

    rows = []
    times, iters = [], []
    for d in successes:
        r = _parse_success(d)
        rows.append(r)
        if isinstance(r.get("time"), (int, float)):
            times.append(r["time"])
        if isinstance(r.get("iterations"), int):
            iters.append(r["iterations"])

    reason_counts = {}
    for d in failures:
        st = _read_status(d)
        reason = st.get("reason") or "unknown"
        key = _reason_bucket(reason)
        reason_counts[key] = reason_counts.get(key, 0) + 1
        rows.append({"id": d.name, "status": "failure", "reason": reason,
                     "time": st.get("wall_seconds")})

    # CSV
    csv_path = REPORT_DIR / "results.csv"
    cols = ["id", "status", "time", "iterations", "max_energy", "avg_energy",
            "verts", "tets", "reason"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=lambda x: x["id"]):
            w.writerow(r)

    # Summary
    total = len(successes) + len(failures)
    lines = []
    lines.append("=" * 64)
    lines.append("simwild / Thingi10K batch report")
    lines.append("=" * 64)
    lines.append(f"output:     {OUT_DIR}")
    lines.append(f"processed:  {total}")
    lines.append(f"  success:  {len(successes)}")
    lines.append(f"  failure:  {len(failures)}")
    if total:
        lines.append(f"  success rate: {100.0 * len(successes) / total:.1f}%")
    lines.append("")
    if times:
        times_sorted = sorted(times)
        lines.append(f"run time (s): min {min(times):.2f}  "
                     f"median {times_sorted[len(times_sorted)//2]:.2f}  "
                     f"max {max(times):.2f}  mean {sum(times)/len(times):.2f}")
    if iters:
        lines.append(f"iterations:   min {min(iters)}  "
                     f"max {max(iters)}  mean {sum(iters)/len(iters):.1f}")
    lines.append("")
    lines.append(_histogram(
        "Run time histogram (seconds, successful models):",
        times, [1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600]))
    lines.append(_histogram(
        "Iteration-count histogram (successful models):",
        iters, [1, 2, 3, 5, 8, 12, 20, 40, 80], fmt="{:g}"))
    lines.append("Failures by reason:")
    if reason_counts:
        for k in sorted(reason_counts, key=lambda x: -reason_counts[x]):
            lines.append(f"  {reason_counts[k]:>6}  {k}")
    else:
        lines.append("  (none)")
    lines.append("")
    lines.append(f"per-model CSV: {csv_path}")

    summary = "\n".join(lines) + "\n"
    (REPORT_DIR / "summary.txt").write_text(summary)
    print(summary, flush=True)

    html_path = generate_html_report(
        rows,
        {"generated": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
         "banner": os.environ.get("SIMWILD_REPORT_BANNER", "")},
        REPORT_DIR,
        OUT_DIR,
        DATASET_DIR,
        MESH_EXTENSIONS,
    )
    print("HTML report:   {}".format(html_path), flush=True)


def _reason_bucket(reason):
    r = reason.lower()
    if "timeout" in r:
        return "timeout"
    if "oom" in r:
        return f"out of memory (MemoryMax={MEM_GB}G)"
    if "sigsegv" in r or "signal 11" in r:
        return "segfault (likely empty/degenerate input)"
    if "signal" in r or "killed" in r:
        return "killed by signal"
    if "nonzero" in r:
        return "nonzero exit"
    return reason or "unknown"


# --------------------------------------------------------------------------- #
# Model selection
# --------------------------------------------------------------------------- #
def _order_models(meshes):
    """Order the work list. Only matters when the run is truncated (LIMIT / a stop).

    smallest: cheapest first, so a partial run covers as many models as possible and
              the timeout mostly bites the giant meshes, which come last. This is the
              right order for the full sweep.
    spread:   walk the size-sorted list at a stride, so a truncated run samples the
              whole size range. A 100-model trial ordered 'smallest' tells you almost
              nothing -- the 100 smallest meshes in Thingi10K are trivial.
    random:   uniform sample, seeded by SIMWILD_SEED.
    name:     filename order. Size is uncorrelated with the name, so the expensive
              models are spread through the run instead of all landing at the end --
              which keeps the machine busy to the finish and makes progress linear
              rather than front-loaded. This is what the 2D runner defaults to.
    """
    if SAMPLE == "name":
        return sorted(meshes, key=lambda p: p.name)
    by_size = sorted(meshes, key=lambda p: p.stat().st_size)
    if SAMPLE == "smallest":
        return by_size
    if SAMPLE == "random":
        shuffled = list(by_size)
        random.Random(SEED).shuffle(shuffled)
        return shuffled
    if SAMPLE == "spread":
        n = len(by_size)
        take = LIMIT if LIMIT else n
        take = max(1, min(take, n))
        stride = n / take
        picked_idx = sorted({min(n - 1, int(i * stride)) for i in range(take)})
        picked = [by_size[i] for i in picked_idx]
        # keep the rest behind the sample so a resume still has work to do
        rest = [m for i, m in enumerate(by_size) if i not in set(picked_idx)]
        return picked + rest
    sys.exit(f"unknown SIMWILD_SAMPLE: {SAMPLE!r} (name | smallest | spread | random)")


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def run_sweep():
    for d in (SUCCESS_DIR, FAILURE_DIR, WORK_DIR, REPORT_DIR):
        d.mkdir(parents=True, exist_ok=True)

    if REPORT_ONLY:
        generate_report()
        return

    if not WMTK_APP.exists():
        sys.exit(f"wmtk_app not found: {WMTK_APP}")
    if not DATASET_DIR.exists():
        sys.exit(f"dataset not found: {DATASET_DIR}")
    if not _memory_capped_available():
        sys.exit(f"cannot create a systemd scope with MemoryMax={MEM_GB}G -- the user "
                 f"manager is not usable here. Set SIMWILD_MEM_GB=0 to run uncapped.")

    # Clean any scratch left behind by a killed run.
    for leftover in WORK_DIR.glob("*"):
        shutil.rmtree(leftover, ignore_errors=True)

    meshes = []
    for pat in MESH_EXTENSIONS:
        meshes.extend(DATASET_DIR.glob(pat))
    ordered = _order_models(meshes)
    todo = [m for m in ordered if not already_done(m.stem)]

    cores = os.cpu_count() or 1
    warn = "  (WARNING: parallel*threads exceeds cores; expect oversubscription)" \
        if PARALLEL * THREADS > cores else ""
    print(f"{len(meshes)} meshes, {len(meshes) - len(todo)} already done, "
          f"{len(todo)} to process.\n"
          f"parallel {PARALLEL} x threads {THREADS} (cores {cores}){warn}, "
          f"timeout {JOB_TIMEOUT}s, "
          f"memory {'uncapped' if MEM_GB <= 0 else str(MEM_GB) + 'G/model'}, "
          f"limit {LIMIT or 'none'}, sample {SAMPLE}.\n"
          f"output: {OUT_DIR}", flush=True)

    PIDFILE.write_text(str(os.getpid()))
    processed = 0
    try:
        # Bounded pool: keep PARALLEL models in flight, feeding a new one whenever a
        # slot frees, and stop feeding once a stop is requested (or the limit is hit).
        it = iter(todo)
        inflight = {}
        with ThreadPoolExecutor(max_workers=PARALLEL) as ex:
            def submit_next():
                nonlocal processed
                if _STOP.is_set() or (LIMIT and processed >= LIMIT):
                    return
                mesh = next(it, None)
                if mesh is None:
                    return
                inflight[ex.submit(run_one, mesh)] = mesh
                processed += 1

            for _ in range(PARALLEL):
                submit_next()
            while inflight:
                done, _ = futures_wait(list(inflight), return_when=FIRST_COMPLETED)
                for fut in done:
                    inflight.pop(fut)
                    try:
                        fut.result()
                    except Exception as e:  # a worker bug must not wedge the pool
                        print(f"  !! worker error: {e}", flush=True)
                    submit_next()
    finally:
        PIDFILE.unlink(missing_ok=True)
        note = " (stopped)" if _STOP.is_set() else ""
        print(f"\nsubmitted {processed} model(s) this run{note}. writing report...",
              flush=True)
        generate_report()


def stop_sweep():
    """Signal a running sweep to stop cleanly (via its pidfile)."""
    if not PIDFILE.exists():
        print("no sweep.pid found -- is a sweep running?")
        return
    try:
        pid = int(PIDFILE.read_text().strip())
    except ValueError:
        print("sweep.pid is unreadable; remove it by hand.")
        return
    try:
        os.kill(pid, signal.SIGTERM)
        print(f"sent SIGTERM to sweep {pid}: it will abandon in-flight models "
              f"(they resume later), write the report, and exit.")
    except ProcessLookupError:
        print(f"sweep {pid} is not running (stale pidfile); removing it.")
        PIDFILE.unlink(missing_ok=True)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "stop":
        stop_sweep()
    else:
        run_sweep()
