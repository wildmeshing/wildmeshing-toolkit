#!/usr/bin/env python3
"""Deterministic golden-hash integration tests for the WildMeshing Toolkit.

The list of integration tests lives in ``data/integration_tests/integration_tests.json``.
Each entry names a per-test config JSON (e.g. ``qslim_octocat.json``) that is run
through the ``wmtk_app`` CLI. This driver runs each test in an isolated working
directory, forcing single-threaded execution, then SHA-256 hashes every output
file the run produces. Those hashes are the golden reference.

Two modes:

* ``--generate`` (bless): run the selected tests and store their output hashes
  back into ``integration_tests.json``. Use ``--only a.json,b.json`` to bless a
  subset (the incremental workflow), or ``--all`` to (re)bless everything.
* ``--check`` (CI): run the blessed tests, recompute the hashes and compare them
  against the stored reference. Exits non-zero on any mismatch (wrong hash,
  missing output, extra output) or if a test crashes. This is what ctest runs.

Reproducibility relies on (a) single-threaded execution (removes parallel
operation-ordering non-determinism) and (b) building the toolkit with
``-DWMTK_FP_STRICT=ON`` (consistent floating-point across compilers/OS). ``.log``
files are ignored because they contain timestamps.

Schema. Entries in the ``integration_tests`` list are either:

* a plain string ``"qslim_octocat.json"`` -- not yet blessed; the test is run for
  termination only (no hash check), or
* an object ``{"file": "qslim_octocat.json", "hashes": {"out.obj": "<sha256>"}}``
  -- blessed; its outputs are hash-checked.

This keeps the per-test config JSONs untouched (they are schema-validated by the
components) and lets tests be added to the golden set one at a time.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
DATA_DIR = Path(
    os.environ.get("WMTK_DATA_DIR", REPO_ROOT / "data" / "integration_tests")
)
TESTS_JSON = DATA_DIR / "integration_tests.json"

# Output files whose bytes are not reproducible (timestamps etc.) are ignored.
IGNORED_SUFFIXES = {".log"}


# --------------------------------------------------------------------------- #
# integration_tests.json access
# --------------------------------------------------------------------------- #
def load_tests_doc() -> dict:
    with open(TESTS_JSON) as f:
        return json.load(f)


def save_tests_doc(doc: dict) -> None:
    with open(TESTS_JSON, "w") as f:
        json.dump(doc, f, indent=4)
        f.write("\n")


def entry_file(entry) -> str:
    """The per-test config filename for a list entry (string or object)."""
    if isinstance(entry, str):
        return entry
    return entry["file"]


def entry_hashes(entry):
    """Stored golden hashes for an entry, or None if not blessed."""
    if isinstance(entry, dict):
        return entry.get("hashes")
    return None


# --------------------------------------------------------------------------- #
# running a single test
# --------------------------------------------------------------------------- #
def find_app(explicit: str | None) -> Path:
    if explicit:
        p = Path(explicit)
        if not p.exists():
            sys.exit(f"wmtk_app not found at --app {p}")
        return p
    env = os.environ.get("WMTK_APP")
    if env and Path(env).exists():
        return Path(env)
    for cand in [
        REPO_ROOT / "build" / "app" / "wmtk_app",
        REPO_ROOT / "build" / "Release" / "app" / "wmtk_app",
        REPO_ROOT / "build" / "Debug" / "app" / "wmtk_app",
    ]:
        if cand.exists():
            return cand
    sys.exit(
        "Could not locate wmtk_app. Build it (-DWMTK_BUILD_INTEGRATION_TESTS is not"
        " needed for the app) or pass --app / set WMTK_APP."
    )


def _absolutize_inputs(cfg: dict, cfg_dir: Path) -> None:
    """Rewrite the config's input path(s) to absolute so the run is independent
    of the working directory (which we relocate to an isolated temp dir)."""
    if "input" not in cfg:
        return
    inp = cfg["input"]

    def abso(p):
        p = Path(p)
        return str(p if p.is_absolute() else (cfg_dir / p).resolve())

    if isinstance(inp, list):
        cfg["input"] = [abso(p) for p in inp]
    else:
        cfg["input"] = abso(inp)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_and_hash(app: Path, test_file: str, threads: int, verbose: bool):
    """Run one integration test in isolation. Returns (hashes, log) where hashes
    is an ordered {relative_output_path: sha256hex} dict. Raises on nonzero exit."""
    src_cfg_path = DATA_DIR / test_file
    with open(src_cfg_path) as f:
        cfg = json.load(f)

    # Force determinism and dir-independence.
    cfg["num_threads"] = threads
    _absolutize_inputs(cfg, DATA_DIR)

    with tempfile.TemporaryDirectory(prefix="wmtk_it_") as td:
        td = Path(td)
        workdir = td / "work"
        workdir.mkdir()
        cfg_path = td / "config.json"  # kept out of workdir so it is not hashed
        with open(cfg_path, "w") as f:
            json.dump(cfg, f)

        proc = subprocess.run(
            [str(app), "-j", str(cfg_path)],
            cwd=workdir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if proc.returncode != 0:
            raise RuntimeError(
                f"{test_file}: wmtk_app exited {proc.returncode}\n{proc.stdout}"
            )

        hashes: dict[str, str] = {}
        for p in sorted(workdir.rglob("*")):
            if not p.is_file():
                continue
            if p.suffix in IGNORED_SUFFIXES:
                continue
            rel = p.relative_to(workdir).as_posix()
            hashes[rel] = sha256_file(p)
    if verbose:
        print(proc.stdout)
    return hashes


# --------------------------------------------------------------------------- #
# modes
# --------------------------------------------------------------------------- #
def cmd_generate(args) -> int:
    doc = load_tests_doc()
    tests = doc["integration_tests"]
    app = find_app(args.app)

    if args.only:
        selected = set(args.only.split(","))
    elif args.all:
        selected = {entry_file(e) for e in tests}
    else:
        selected = {entry_file(e) for e in tests if entry_hashes(e) is not None}
    if not selected:
        print("Nothing to generate (use --only <files> or --all).")
        return 0

    failures = []
    for i, entry in enumerate(tests):
        name = entry_file(entry)
        if name not in selected:
            continue
        print(f"[generate] {name} ...", flush=True)
        try:
            hashes = run_and_hash(app, name, args.threads, args.verbose)
        except Exception as e:
            print(f"  FAILED: {e}")
            failures.append(name)
            continue
        tests[i] = {"file": name, "hashes": hashes}
        print(f"  blessed {len(hashes)} output(s): {', '.join(hashes) or '(none)'}")

    save_tests_doc(doc)
    if failures:
        print(f"\n{len(failures)} test(s) failed to generate: {', '.join(failures)}")
        return 1
    return 0


def cmd_check(args) -> int:
    doc = load_tests_doc()
    tests = doc["integration_tests"]
    app = find_app(args.app)

    only = set(args.only.split(",")) if args.only else None
    n_checked = 0
    n_ran = 0
    mismatches = []
    for entry in tests:
        name = entry_file(entry)
        if only is not None and name not in only:
            continue
        golden = entry_hashes(entry)
        # By default only the blessed tests are exercised (the golden-hash check).
        # --include-unblessed also runs the not-yet-blessed tests for termination,
        # like the C++ integration test does.
        if golden is None and not args.include_unblessed:
            continue
        try:
            hashes = run_and_hash(app, name, args.threads, args.verbose)
        except Exception as e:
            print(f"[FAIL] {name}: {e}", flush=True)
            mismatches.append(name)
            continue
        n_ran += 1
        if golden is None:
            print(f"[run ] {name} (not blessed, termination only)", flush=True)
            continue
        n_checked += 1
        if hashes == golden:
            print(f"[ ok ] {name} ({len(hashes)} output(s))", flush=True)
        else:
            mismatches.append(name)
            print(f"[FAIL] {name}: output hashes differ", flush=True)
            _report_diff(golden, hashes)

    print(f"\nRan {n_ran} test(s); hash-checked {n_checked}; {len(mismatches)} failure(s).")
    return 1 if mismatches else 0


def _report_diff(golden: dict, got: dict) -> None:
    gk, ck = set(golden), set(got)
    for f in sorted(gk - ck):
        print(f"        missing output: {f}")
    for f in sorted(ck - gk):
        print(f"        unexpected output: {f}")
    for f in sorted(gk & ck):
        if golden[f] != got[f]:
            print(f"        differs: {f}\n          expected {golden[f]}\n          got      {got[f]}")


def cmd_list(args) -> int:
    doc = load_tests_doc()
    blessed, unblessed = [], []
    for e in doc["integration_tests"]:
        (blessed if entry_hashes(e) is not None else unblessed).append(entry_file(e))
    print(f"Blessed ({len(blessed)}):")
    for n in blessed:
        print(f"  {n}")
    print(f"Not blessed ({len(unblessed)}):")
    for n in unblessed:
        print(f"  {n}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--app", help="path to wmtk_app (default: build/app/wmtk_app or $WMTK_APP)")
    ap.add_argument("--threads", type=int, default=1, help="num_threads to force (default 1)")
    ap.add_argument("--verbose", action="store_true", help="print each run's stdout")
    sub = ap.add_subparsers(dest="mode", required=True)

    g = sub.add_parser("generate", help="bless: run tests and store output hashes")
    g.add_argument("--only", help="comma-separated config filenames to (re)bless")
    g.add_argument("--all", action="store_true", help="bless every test")
    g.set_defaults(func=cmd_generate)

    c = sub.add_parser("check", help="run tests and verify hashes (exit != 0 on mismatch)")
    c.add_argument("--only", help="comma-separated config filenames to check")
    c.add_argument(
        "--include-unblessed",
        action="store_true",
        help="also run not-yet-blessed tests for termination (default: blessed only)",
    )
    c.set_defaults(func=cmd_check)

    l = sub.add_parser("list", help="show blessed vs not-yet-blessed tests")
    l.set_defaults(func=cmd_list)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
