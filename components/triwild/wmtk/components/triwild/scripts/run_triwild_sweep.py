#!/usr/bin/env python3
"""
Batch-run triwild over a large 2D curve dataset. See README.md.

Written for the 20k 2D dataset on kirby.cs.nyu.edu, whose layout is the default for
TRIWILD_ROOT; set that variable to run it anywhere. The 2D counterpart of the
Thingi10K tetwild runner, and deliberately the same contract: run it with no
arguments to start (or resume) the sweep; it skips every model already in success/
or failure/. Stop it cleanly with:

    run_triwild_sweep.py stop

which signals the running sweep to abandon whatever is in flight (those models are
left unpublished, so a later resume reprocesses them -- no spurious failures), drain,
write the report, and exit.

What differs from the 3D runner:
  * paths hang off TRIWILD_ROOT;
  * the dataset is .obj only -- triwild reads segment networks ("v" + "l" lines), not
    triangle soups, so the other extensions are not applicable;
  * write_vtu is forced OFF. It defaults to true now (parity with tetwild), but the
    sweep prunes .vtu anyway, and 20k of them would be written only to be deleted.
    The .msh is the actual output;
  * skip_winding_number is forced ON. With filter="none" the winding number only feeds
    the MSH group tags, which this sweep does not read, and it is brute-force O(#queries
    x #segments) in 2D -- it accounted for 95% of the timeouts in the previous run;
  * DEBUG_hausdorff is forced ON, and the report separates its two directions:
    CONTAINMENT d(output->input), the envelope invariant, which is the only one bounded
    by eps, and COVERAGE d(input->output), which nothing promises and which is reported
    without a threshold. Runs made before that direction was corrected stored coverage
    under the name "hausdorff" and carry no "coverage" key; the report detects them by
    that absence, labels them metric=legacy-coverage, and keeps them out of the
    containment statistics rather than pooling two different quantities;
  * the report buckets the two triwild-specific failures -- the arrangement orientation
    error and the envelope sanity-check error -- instead of lumping them into
    "nonzero exit";
  * env vars are TRIWILD_*, not TETWILD_*, so a 2D and a 3D sweep can run side by side
    on the same machine without one's settings leaking into the other.

Per model:
  * write a triwild JSON (defaults from whatever branch build/ was built from);
  * run wmtk_app in a scratch dir under the output volume, capped in time and memory;
  * exit 0                    -> move the run into  <OUT>/success/<id>/
    nonzero / timeout / OOM   -> move the run into  <OUT>/failure/<id>/  (with a reason)

Configuration -- environment variables:
    TRIWILD_ROOT          sweep root: build/, data/, runs/  (default the kirby path)
    TRIWILD_OUT           output directory            (default <root>/runs/full)
    TRIWILD_PARALLEL      models to run concurrently  (default 16)
    TRIWILD_THREADS       threads per model           (default 8)
    TRIWILD_JOB_TIMEOUT   per-model seconds           (default 3600 = 1h)
    TRIWILD_MEM_GB        per-model memory cap, GB    (default 128, 0 disables)
    TRIWILD_LIMIT         process at most N new models (default 0 = all)
    TRIWILD_SAMPLE        name | smallest | spread | random  (default name)
    TRIWILD_SEED          seed for TRIWILD_SAMPLE=random  (default 0)
    TRIWILD_REPORT_ONLY   if set, only regenerate the report and exit

Memory note: the cap is PER MODEL, not a budget for the sweep. 16 x 128G is far more
than kirby has, so the cap is a runaway-killer, not admission control -- it stops one
pathological input from swapping the box, and does nothing in the normal case. It
matters more here than in 3D: the dataset's file sizes span 2.7 KB to 1.6 GB, and the
largest inputs are read whole before anything else happens.
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
# Hardcoded configuration
# --------------------------------------------------------------------------- #
# Everything hangs off one root: build/ (the wmtk_app to test), data/ (the .obj
# dataset) and runs/ (the outputs). Override with TRIWILD_ROOT to run this anywhere;
# the default is the kirby layout it was written for.
SWEEP_ROOT = Path(os.environ.get("TRIWILD_ROOT", "/u/3/daniele/triwild-sweep"))
WMTK_APP = SWEEP_ROOT / "build/app/wmtk_app"
DATASET_DIR = SWEEP_ROOT / "data"
OUT_DIR = Path(os.environ.get("TRIWILD_OUT", str(SWEEP_ROOT / "runs/full")))

# triwild input is a 2D segment network: an .obj carrying "v" and "l" lines. The
# dataset is uniformly .obj, and every filename stem is unique, so the stem is a safe
# per-model id.
MESH_EXTENSIONS = ("*.obj",)

PARALLEL = max(1, int(os.environ.get("TRIWILD_PARALLEL", "16")))
THREADS = max(1, int(os.environ.get("TRIWILD_THREADS", "8")))
JOB_TIMEOUT = int(os.environ.get("TRIWILD_JOB_TIMEOUT", "3600"))  # seconds, 1h
MEM_GB = int(os.environ.get("TRIWILD_MEM_GB", "128"))  # 0 = no cap
LIMIT = int(os.environ.get("TRIWILD_LIMIT", "0"))  # 0 = no limit
SAMPLE = os.environ.get("TRIWILD_SAMPLE", "name")
SEED = int(os.environ.get("TRIWILD_SEED", "0"))
REPORT_ONLY = bool(os.environ.get("TRIWILD_REPORT_ONLY"))

# Only the knobs the SWEEP needs, not the ones under test. Everything else comes from
# the spec defaults of whatever branch build/ was built from -- which is the point of
# running this on triwild-sweep, so do not pin eps_rel, stop_energy or the simplify_*
# parameters here.
PARAMS = {
    "application": "triwild",
    "filter": "none",
    "num_threads": THREADS,
    # Raised from the spec default of 20. Note this changes what "converged" MEANS, not just
    # how long it takes: models processed before this point were held to 20 and later ones to
    # 100, so max_energy is not comparable across the two halves of the run.
    "stop_energy": 100,
    "write_vtu": False,  # pruned anyway; do not spend the disk and the time
    "DEBUG_hausdorff": True,  # the open 2D question -- collect it for every model
    # The finalize-phase winding number, which with filter="none" only feeds the MSH group
    # tags. It is not skipped automatically -- filter="none" is necessary but not sufficient,
    # in 2D and 3D alike -- and it dominates the run on big inputs: libigl has a hierarchical
    # accelerator for 3D triangles but none in 2D, so winding_number_2d is a brute-force
    # O(#queries x #segments) sweep, ~2M faces x ~400k segments on the models that suffer.
    #
    # It was 95% of this sweep's failures: 232 of 245 timeouts died in that phase with the
    # mesh ALREADY converged (median last max energy 19.99 against a target of 20, and 243 of
    # 245 still improving when killed). Model 177011 finishes in 567s with this on and times
    # out at 1800s without it.
    #
    # The cost is the group tags: every face lands in the untagged group. This sweep does not
    # read them.
    "skip_winding_number": True,
    # Deliberately NOT enabled. It is a diagnostic on a property the pipeline does not
    # guarantee, and the sweep is for throughput. Consequence: models processed before this
    # was turned off carry features_retained / features_total / features_worst_ratio and
    # later ones do not, so the report treats those columns as optional.
}

# Which of a run's output files to keep. The .msh is the actual mesh. Keep it, the
# text (logs / report / config), and any .obj the run writes.
KEEP_GLOBS = ["*.msh", "*.obj", "*.log", "*.json", "status.txt"]

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
# dependencies reserve large virtual ranges they never fault in, so an address-space
# limit fires on models that are nowhere near the memory it is meant to bound.
# MemoryMax is real usage. MemorySwapMax=0 keeps a bloated model from dragging the
# whole machine into swap instead of dying.
#
# The scope is a child in our own process group, so the existing killpg path still
# terminates it; the cap only changes how the kernel treats its memory.
# --------------------------------------------------------------------------- #
def _mem_wrapper(model_id):
    if MEM_GB <= 0:
        return []
    return [
        "systemd-run", "--user", "--scope", "--quiet",
        f"--unit=trw-{model_id}-{os.getpid()}",
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
    """Run triwild on one mesh. Returns 'success' / 'failure' / 'stopped'."""
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
            # the config's parent_path(), and older builds threw on the empty path.
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

    _prune(work)
    dest = (SUCCESS_DIR if status == "success" else FAILURE_DIR) / model_id
    if dest.exists():
        shutil.rmtree(dest, ignore_errors=True)
    os.replace(work, dest)  # atomic rename within the same filesystem

    print(f"  -> {status}: {model_id}  ({elapsed:.1f}s"
          + (f", {reason}" if reason else "") + ")", flush=True)
    return status


# The toolkit reports a fatal condition by throwing std::runtime_error out of main, so
# the useful part of a failure is that message, not the exit code (which is just
# SIGABRT for every one of them). Pull it out of the console so the report can group by
# cause. Three patterns, because the two standard libraries print differently:
#
#   libstdc++ (kirby):  terminate called after throwing an instance of 'std::runtime_error'
#                         what():  Tets with different orientations in the input!
#   libc++ (macOS):     libc++abi: terminating due to uncaught exception of type
#                       std::runtime_error: Tets with different orientations in the input!
#
# and the wmtk logger's own [error] line, which precedes both and survives when the
# abort message does not (a hard signal, a truncated console).
_THROW_RES = [
    re.compile(r"what\(\):\s*(.+)"),
    re.compile(r"(?:runtime_error|logic_error|bad_alloc)\s*:\s*(.+)"),
    re.compile(r"\[wmtk\]\s*\[error\]\s*(.+)"),
]


def _describe_exit(code, work):
    """Human-readable reason for a nonzero exit, distinguishing the OOM kill."""
    console = work / "console.log"
    if console.exists():
        text = console.read_text(errors="ignore")
        tail = text[-8000:]
        low = tail.lower()
        if "oom" in low or "out of memory" in low:
            return f"OOM (MemoryMax={MEM_GB}G)"
        for rx in _THROW_RES:
            hits = rx.findall(tail)
            if hits:
                # the last one is the message that actually terminated the run
                return hits[-1].strip()
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
IT_RE = re.compile(r"========it (\d+)========")
SIMP_RE = re.compile(r"input simplification: #V (\d+) -> (\d+), #E (\d+) -> (\d+)")


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
    """time, iterations, energies, hausdorff for one successful model."""
    row = {"id": d.name, "status": "success"}
    rep = d / "report.json"
    if rep.exists():
        try:
            j = json.loads(rep.read_text())
            row["max_energy"] = j.get("max_energy")
            row["avg_energy"] = j.get("avg_energy")
            row["verts"] = j.get("#v")
            row["tris"] = j.get("#t")
            row["all_rounded"] = j.get("all_rounded")
            # "hausdorff" is CONTAINMENT, d(output -> input): the envelope invariant, and
            # the only direction bounded by eps. "coverage" is d(input -> output), which
            # nothing promises -- simplification may remove detail and the arrangement may
            # drop segments. Reporting coverage against eps is what made ~60% of models
            # look like violations; see PR #982 and tetwild #967 for the same mistake in 3D.
            h, cov, eps = j.get("hausdorff"), j.get("coverage"), j.get("eps")
            # Legacy detection. Before the direction was corrected, "hausdorff" held
            # COVERAGE and there was no "coverage" key. Those runs' meshes are unaffected
            # (the change is diagnostic-only), but their number means something different,
            # so it must not be pooled with the new one -- averaging the two would show
            # phantom containment violations. Absence of "coverage" is the marker.
            if cov is None:
                row["metric"] = "legacy-coverage"
                row["coverage"] = h
                if isinstance(eps, (int, float)) and eps > 0 and isinstance(h, (int, float)) \
                        and h >= 0:
                    row["coverage_over_eps"] = h / eps
                h = None  # no containment figure exists for this run
            else:
                row["metric"] = "both"
                row["containment"] = h
                row["coverage"] = cov
            # Only the ratios are comparable across the dataset: the distances are in model
            # units and eps scales with the bounding box. A negative value is the "output has
            # no tracked edges" sentinel, not a distance.
            if isinstance(eps, (int, float)) and eps > 0:
                if isinstance(h, (int, float)) and h >= 0:
                    row["containment_over_eps"] = h / eps
                if isinstance(cov, (int, float)) and cov >= 0:
                    row["coverage_over_eps"] = cov / eps
        except json.JSONDecodeError:
            pass
    log = d / "out.log"
    if log.exists():
        text = log.read_text(errors="ignore")
        row["iterations"] = len(IT_RE.findall(text))
        m = SIMP_RE.search(text)
        if m:
            v0, v1 = int(m.group(1)), int(m.group(2))
            row["input_verts"] = v0
            row["simplify_ratio"] = (v1 / v0) if v0 else None
    st = _read_status(d)
    # triwild's report.json does not carry a runtime (tetwild's does), so the wall
    # clock recorded here is the only timing available. It includes process startup
    # and the read of what can be a 1.6 GB .obj, which is the honest number anyway.
    if st.get("wall_seconds"):
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

# Slots 1-3 of the validated categorical palette, plus the status colors. Only three
# categorical slots are used, which is the all-pairs-safe prefix; every chart here is
# single-series anyway, so hue never carries meaning on its own.
_CSS = """
:root {
  color-scheme: light;
  --surface-1: #fcfcfb; --plane: #f9f9f7;
  --ink-1: #0b0b0b; --ink-2: #52514e; --ink-muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7;
  --series-1: #2a78d6; --series-2: #eb6834; --series-3: #1baf7a;
  --good: #0ca30c; --critical: #d03b3b;
  --ramp-100: #cde2fb; --ramp-450: #2a78d6; --ramp-600: #184f95;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --surface-1: #1a1a19; --plane: #0d0d0d;
    --ink-1: #ffffff; --ink-2: #c3c2b7; --ink-muted: #898781;
    --grid: #2c2c2a; --axis: #383835;
    --series-1: #3987e5; --series-2: #d95926; --series-3: #199e70;
    --ramp-100: #184f95; --ramp-450: #3987e5; --ramp-600: #86b6ef;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --surface-1: #1a1a19; --plane: #0d0d0d;
  --ink-1: #ffffff; --ink-2: #c3c2b7; --ink-muted: #898781;
  --grid: #2c2c2a; --axis: #383835;
  --series-1: #3987e5; --series-2: #d95926; --series-3: #199e70;
  --ramp-100: #184f95; --ramp-450: #3987e5; --ramp-600: #86b6ef;
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
.sub { color: var(--ink-2); font-size: 0.9rem; margin: 0 0 1.6rem; }
.sub code { font-size: 0.85em; color: var(--ink-muted); }
.note {
  color: var(--ink-2); font-size: 0.87rem; margin: 0.55rem 0 0;
  max-width: 74ch;
}

/* hero stats */
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

/* charts */
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
.bar {
  height: 15px; border-radius: 0 4px 4px 0; background: var(--series-1);
  min-width: 2px; transition: filter .12s;
}
.track:hover .bar { filter: brightness(1.12); }
.bar.zero { background: none; border: 1px dashed var(--axis); height: 13px; min-width: 12px; }
.bar.warnzone { background: var(--critical); }
.cnt {
  font-size: 0.76rem; color: var(--ink-2); margin-left: 0.45rem;
  font-variant-numeric: tabular-nums; white-space: nowrap;
}
.grid2 { display: grid; grid-template-columns: repeat(auto-fit, minmax(430px, 1fr)); gap: 0.9rem; }

/* table */
.tblwrap { overflow-x: auto; background: var(--surface-1);
           border: 1px solid var(--grid); border-radius: 10px; }
table { border-collapse: collapse; width: 100%; font-size: 0.86rem; }
th, td { padding: 0.42rem 0.75rem; text-align: right; white-space: nowrap; }
th {
  font-size: 0.71rem; text-transform: uppercase; letter-spacing: 0.05em;
  color: var(--ink-muted); font-weight: 600; border-bottom: 1px solid var(--grid);
  position: sticky; top: 0; background: var(--surface-1);
}
td { border-bottom: 1px solid var(--grid); font-variant-numeric: tabular-nums; }
tr:last-child td { border-bottom: none; }
th:first-child, td:first-child { text-align: left; }
details { margin-top: 0.7rem; }
summary {
  cursor: pointer; color: var(--ink-2); font-size: 0.85rem;
  padding: 0.3rem 0; user-select: none;
}
summary:hover { color: var(--ink-1); }

/* tooltip */
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
      tip.style.left = x + 'px';
      tip.style.top = y + 'px';
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
        if v != v:  # NaN
            return "-"
        if v >= 1e5 or (v and abs(v) < 1e-3):
            return "{:.3g}".format(v)
        return "{:.{}f}".format(v, nd)
    return str(v)


def _bucket(values, edges):
    """Counts per bucket, plus the label for each. Last bucket is open-ended."""
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


def _html_hist(values, edges, unit, danger_from=None):
    """Horizontal bar chart. danger_from marks buckets at/above that edge index red."""
    if not values:
        return '<p class="note">(no data)</p>'
    labels, counts = _bucket(values, edges)
    peak = max(counts) or 1
    total = sum(counts) or 1
    out = ['<div class="chart">']
    for i, (lab, c) in enumerate(zip(labels, counts)):
        pct = 100.0 * c / total
        cls = "bar"
        if c == 0:
            cls += " zero"
        elif danger_from is not None and i >= danger_from:
            cls += " warnzone"
        w = 100.0 * c / peak
        tip = "{} {}: {} models ({:.1f}%)".format(lab, unit, c, pct)
        out.append('<div class="lab">{}</div>'.format(_esc(lab)))
        out.append(
            '<div class="track" data-tip="{}">'
            '<div class="{}" style="width:{:.2f}%"></div>'
            '<span class="cnt">{}</span></div>'.format(
                _esc(tip), cls, max(w, 0.0) if c else 0.0, "{:,}".format(c)))
    out.append("</div>")
    return "".join(out)


def _stat(k, v, u="", cls=""):
    return ('<div class="stat {}"><div class="k">{}</div><div class="v">{}</div>'
            '<div class="u">{}</div></div>').format(cls, _esc(k), _esc(v), _esc(u))


def _table(headers, rows, empty="(none)"):
    if not rows:
        return '<p class="note">{}</p>'.format(_esc(empty))
    h = "".join("<th>{}</th>".format(_esc(x)) for x in headers)
    body = []
    for r in rows:
        body.append("<tr>" + "".join("<td>{}</td>".format(_esc(c)) for c in r) + "</tr>")
    return ('<div class="tblwrap"><table><thead><tr>{}</tr></thead>'
            "<tbody>{}</tbody></table></div>").format(h, "".join(body))


def generate_html_report(rows, stats, report_dir, out_dir):
    """Write report/index.html. `rows` are the same dicts the CSV is built from."""
    ok = [r for r in rows if r.get("status") == "success"]
    bad = [r for r in rows if r.get("status") != "success"]
    total = len(rows)

    def col(key, src=None):
        return [r[key] for r in (src if src is not None else ok)
                if isinstance(r.get(key), (int, float))]

    times, iters = col("time"), col("iterations")
    contr, covr = col("containment_over_eps"), col("coverage_over_eps")
    energies = col("max_energy")
    unrounded = sum(1 for r in ok if r.get("all_rounded") is False)

    def med(xs):
        return sorted(xs)[len(xs) // 2] if xs else None

    rate = (100.0 * len(ok) / total) if total else 0.0
    over1 = sum(1 for v in contr if v > 1.0)

    h = []
    h.append("<!doctype html><html lang='en'><head><meta charset='utf-8'>")
    h.append("<meta name='viewport' content='width=device-width,initial-scale=1'>")
    h.append("<title>triwild 2D sweep report</title>")
    h.append("<style>{}</style></head><body><div id='tip'></div><div class='wrap'>".format(_CSS))

    h.append("<h1>triwild &mdash; 20k 2D curve sweep</h1>")
    h.append("<p class='sub'>{} models &middot; <code>{}</code> &middot; generated {}</p>".format(
        "{:,}".format(total), _esc(out_dir), _esc(stats.get("generated", ""))))

    # ---- headline ----
    h.append("<div class='stats'>")
    h.append(_stat("Processed", "{:,}".format(total)))
    h.append(_stat("Success rate", "{:.2f}%".format(rate),
                   "{:,} succeeded".format(len(ok)), "ok" if not bad else ""))
    h.append(_stat("Failures", "{:,}".format(len(bad)),
                   "of {:,}".format(total), "bad" if bad else "ok"))
    h.append(_stat("Un-rounded output", "{:,}".format(unrounded),
                   "of {:,} successes".format(len(ok)), "ok" if not unrounded else "bad"))
    h.append("</div><div class='stats'>")
    h.append(_stat("Median run time", _fmt(med(times), 1), "seconds"))
    h.append(_stat("Max run time", _fmt(max(times), 0) if times else "-", "seconds"))
    h.append(_stat("Median iterations", _fmt(med(iters), 0), "per model"))
    h.append(_stat("Max containment", _fmt(max(contr), 3) if contr else "-",
                   "× eps — invariant is 1.0", "ok" if not over1 else "bad"))
    h.append("</div>")

    # ---- envelope invariant ----
    h.append("<h2>Envelope containment &mdash; the invariant</h2>")
    h.append("<div class='card'>")
    h.append(_html_hist(contr, [0.1, 0.25, 0.5, 0.75, 1.0, 2.0], "× eps", danger_from=4))
    h.append("</div>")
    h.append("<p class='note'><strong>d(output &rarr; input) / eps.</strong> This is the one "
             "quantity the pipeline actually promises: every point of the output lies within "
             "eps of the input. Anything at or above 1.0 (shown red) is a real violation. "
             "Measured: median {}, max {}, over 1.0: <strong>{} of {:,}</strong>.</p>".format(
                 _fmt(med(contr), 3), _fmt(max(contr), 4) if contr else "-",
                 over1, len(contr)))

    # ---- distributions ----
    h.append("<h2>Distributions</h2><div class='grid2'>")
    h.append("<div class='card'><h3 style='margin:0 0 .7rem;font-size:.9rem;'>"
             "Run time (seconds)</h3>{}</div>".format(
                 _html_hist(times, [1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600], "s")))
    h.append("<div class='card'><h3 style='margin:0 0 .7rem;font-size:.9rem;'>"
             "Iterations</h3>{}</div>".format(
                 _html_hist(iters, [1, 2, 3, 5, 8, 12, 20, 40, 80], "iterations")))
    h.append("<div class='card'><h3 style='margin:0 0 .7rem;font-size:.9rem;'>"
             "Final max energy <span style='color:var(--ink-muted);font-weight:400'>"
             "(AMIPS2D floor is 2)</span></h3>{}</div>".format(
                 _html_hist(energies, [5, 10, 20, 21, 50, 100, 101], "energy")))
    h.append("<div class='card'><h3 style='margin:0 0 .7rem;font-size:.9rem;'>"
             "Coverage d(input &rarr; output) / eps "
             "<span style='color:var(--ink-muted);font-weight:400'>(diagnostic)</span>"
             "</h3>{}</div>".format(
                 _html_hist(covr, [0.5, 1, 2, 5, 10, 50, 100], "× eps")))
    h.append("</div>")
    h.append("<p class='note'>Coverage is <em>not</em> bounded by anything and a large value "
             "is not a violation: the simplification is allowed to remove detail and the "
             "arrangement may drop segments, so a high number means the output no longer "
             "covers part of the input. Median {}, max {}. Use <code>DEBUG_euler</code> to see "
             "which curves were lost.</p>".format(
                 _fmt(med(covr), 3), _fmt(max(covr), 4) if covr else "-"))

    # ---- scaling ----
    sized = [r for r in ok if isinstance(r.get("input_verts"), (int, float))
             and isinstance(r.get("time"), (int, float)) and r["input_verts"] > 0]
    if sized:
        h.append("<h2>Run time vs input size</h2>")
        edges = [1e3, 1e4, 1e5, 3e5, 1e6, 3e6]
        buckets = [[] for _ in range(len(edges) + 1)]
        for r in sized:
            for i, e in enumerate(edges):
                if r["input_verts"] < e:
                    buckets[i].append(r)
                    break
            else:
                buckets[-1].append(r)
        labels, _ = _bucket([r["input_verts"] for r in sized], edges)
        trows = []
        for lab, b in zip(labels, buckets):
            if not b:
                continue
            ts = sorted(r["time"] for r in b)
            p95 = ts[min(len(ts) - 1, int(0.95 * len(ts)))]
            its = [r["iterations"] for r in b if isinstance(r.get("iterations"), int)]
            trows.append([lab, "{:,}".format(len(b)), _fmt(ts[len(ts) // 2], 1),
                          _fmt(p95, 1), _fmt(max(ts), 1),
                          _fmt(sum(its) / len(its), 1) if its else "-"])
        h.append(_table(["input vertices", "models", "median s", "p95 s", "max s",
                         "mean iters"], trows))
        h.append("<p class='note'>Wall clock includes process startup and reading the "
                 "<code>.obj</code>, which for the largest inputs is over a gigabyte, so "
                 "the smallest bucket is dominated by fixed cost rather than by the mesh.</p>")

    # ---- slowest ----
    slow = sorted([r for r in ok if isinstance(r.get("time"), (int, float))],
                  key=lambda r: -r["time"])[:25]
    if slow:
        h.append("<h2>Slowest models</h2>")
        h.append(_table(
            ["id", "time s", "iters", "max energy", "#V out", "#T out",
             "input #V", "containment/eps"],
            [[r["id"], _fmt(r.get("time"), 1), r.get("iterations", "-"),
              _fmt(r.get("max_energy"), 3), "{:,}".format(r["verts"]) if r.get("verts") else "-",
              "{:,}".format(r["tris"]) if r.get("tris") else "-",
              "{:,}".format(int(r["input_verts"])) if r.get("input_verts") else "-",
              _fmt(r.get("containment_over_eps"), 3)] for r in slow]))

    # ---- failures ----
    h.append("<h2>Failures</h2>")
    if bad:
        h.append(_table(["id", "reason", "wall s"],
                        [[r["id"], r.get("reason") or "unknown", _fmt(r.get("time"), 1)]
                         for r in sorted(bad, key=lambda r: str(r["id"]))]))
    else:
        h.append("<div class='stats'>{}</div>".format(
            _stat("Failures", "0", "every model produced an output", "ok")))

    h.append("<details><summary>Full per-model data</summary>"
             "<p class='note'>{:,} rows in <code>{}</code> &mdash; one line per model with "
             "time, iterations, energies, element counts and both Hausdorff directions."
             "</p></details>".format(total, _esc(str(report_dir / "results.csv"))))

    h.append("<footer>triwild 2D sweep &middot; {} &middot; "
             "containment is the invariant; coverage is diagnostic."
             "</footer>".format(_esc(stats.get("generated", ""))))
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
    times, iters, contr, covr, energies = [], [], [], [], []
    n_unrounded = 0
    for d in successes:
        r = _parse_success(d)
        rows.append(r)
        if isinstance(r.get("time"), (int, float)):
            times.append(r["time"])
        if isinstance(r.get("iterations"), int):
            iters.append(r["iterations"])
        if isinstance(r.get("containment_over_eps"), (int, float)):
            contr.append(r["containment_over_eps"])
        if isinstance(r.get("coverage_over_eps"), (int, float)):
            covr.append(r["coverage_over_eps"])
        if isinstance(r.get("max_energy"), (int, float)):
            energies.append(r["max_energy"])
        if r.get("all_rounded") is False:
            n_unrounded += 1

    reason_counts = {}
    for d in failures:
        st = _read_status(d)
        reason = st.get("reason") or "unknown"
        # Retroactive repair: a run recorded before the libstdc++ abort format was
        # handled says only "killed by signal 6 (SIGABRT)", which buckets everything
        # fatal together. console.log is kept, so the real message is still there --
        # re-derive rather than make the user re-run 20k models for a report bug.
        if "signal" in reason.lower() and (d / "console.log").exists():
            better = _describe_exit(None, d)
            if better and "signal" not in better.lower():
                reason = better
        key = _reason_bucket(reason)
        reason_counts[key] = reason_counts.get(key, 0) + 1
        rows.append({"id": d.name, "status": "failure", "reason": reason,
                     "time": st.get("wall_seconds")})

    csv_path = REPORT_DIR / "results.csv"
    cols = ["id", "status", "time", "iterations", "max_energy", "avg_energy",
            "verts", "tris", "all_rounded", "metric", "containment",
            "containment_over_eps", "coverage", "coverage_over_eps", "input_verts",
            "simplify_ratio", "reason"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=lambda x: x["id"]):
            w.writerow(r)

    total = len(successes) + len(failures)
    lines = []
    lines.append("=" * 64)
    lines.append("triwild / 20k 2D batch report")
    lines.append("=" * 64)
    lines.append(f"output:     {OUT_DIR}")
    lines.append(f"processed:  {total}")
    lines.append(f"  success:  {len(successes)}")
    lines.append(f"  failure:  {len(failures)}")
    if total:
        lines.append(f"  success rate: {100.0 * len(successes) / total:.1f}%")
    lines.append("")
    if times:
        ts = sorted(times)
        lines.append(f"run time (s): min {min(ts):.2f}  median {ts[len(ts)//2]:.2f}  "
                     f"max {max(ts):.2f}  mean {sum(ts)/len(ts):.2f}")
    if iters:
        lines.append(f"iterations:   min {min(iters)}  max {max(iters)}  "
                     f"mean {sum(iters)/len(iters):.1f}")
    if energies:
        es = sorted(energies)
        lines.append(f"max energy:   median {es[len(es)//2]:.3f}  max {max(es):.4g}   "
                     f"(AMIPS2D floor is 2)")
    if successes:
        lines.append(f"un-rounded output: {n_unrounded} of {len(successes)} successes")
    if contr:
        cs = sorted(contr)
        over = sum(1 for v in contr if v > 1.0)
        lines.append(f"containment d(out->in)/eps: median {cs[len(cs)//2]:.3f}  "
                     f"max {max(cs):.4g}   OVER 1.0: {over}/{len(contr)}")
        lines.append(f"  (the envelope invariant. Anything over 1.0 is a real violation. "
                     f"Measured on {len(contr)} of {len(successes)} successes -- the rest "
                     f"predate the direction fix and have no containment figure.)")
    if covr:
        vs = sorted(covr)
        over = sum(1 for v in covr if v > 1.0)
        lines.append(f"coverage    d(in->out)/eps: median {vs[len(vs)//2]:.3f}  "
                     f"max {max(vs):.4g}   over 1.0: {over}/{len(covr)}")
        lines.append("  (diagnostic only, bounded by nothing: simplification may remove "
                     "detail and the arrangement may drop segments. Large means the output "
                     "no longer covers part of the input. Use DEBUG_euler to see which "
                     "curves were lost.)")
    lines.append("")
    lines.append(_histogram(
        "Run time histogram (seconds, successful models):",
        times, [1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600]))
    lines.append(_histogram(
        "Iteration-count histogram (successful models):",
        iters, [1, 2, 3, 5, 8, 12, 20, 40, 80]))
    lines.append(_histogram(
        "Containment d(output->input) / eps histogram (successful models):",
        contr, [0.1, 0.25, 0.5, 0.75, 1.0, 2.0]))
    lines.append(_histogram(
        "Coverage d(input->output) / eps histogram (successful models, diagnostic):",
        covr, [0.5, 1, 2, 5, 10, 50, 100]))
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
        {"generated": datetime.datetime.now().strftime("%Y-%m-%d %H:%M")},
        REPORT_DIR,
        OUT_DIR,
    )
    print("HTML report:   {}".format(html_path), flush=True)


def _reason_bucket(reason):
    r = reason.lower()
    if "timeout" in r:
        return "timeout"
    if "oom" in r:
        return f"out of memory (MemoryMax={MEM_GB}G)"
    # The two triwild-specific fatals, both known before the sweep started (PR #982).
    if "different orientations" in r:
        return "arrangement: tets with different orientations"
    if "is outside" in r:
        return "init: input edge outside the envelope"
    if "empty output" in r:
        return "empty output after filter"
    if "sigsegv" in r or "signal 11" in r:
        return "segfault"
    if "signal" in r or "killed" in r:
        return "killed by signal"
    if "nonzero" in r:
        return "nonzero exit (no message)"
    return reason or "unknown"


# --------------------------------------------------------------------------- #
# Model selection
# --------------------------------------------------------------------------- #
def _order_models(meshes):
    """Order the work list.

    name:     lexicographic by filename. The default for the full sweep. File size is
              uncorrelated with the name here, so the expensive models are spread evenly
              through the run instead of arriving in one block at the end -- which keeps
              throughput and the failure mix representative at every point, and means an
              interrupted run is still a fair sample rather than "all the easy ones".
    smallest: cheapest first. Maximises how many models a truncated run completes, but
              back-loads every expensive model; with sizes spanning 2.7 KB to 1.6 GB that
              makes the tail of the sweep look nothing like the head.
    spread:   walk the size-sorted list at a stride, so a truncated run samples the whole
              size range. That is what you want from a trial.
    random:   uniform sample, seeded by TRIWILD_SEED.
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
    sys.exit(f"unknown TRIWILD_SAMPLE: {SAMPLE!r} (name | smallest | spread | random)")


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
                 f"manager is not usable here. Set TRIWILD_MEM_GB=0 to run uncapped.")

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
