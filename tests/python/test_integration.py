"""Python integration tests for the WildMeshingToolkit bindings (``wildmeshing``).

These mirror the C++ integration tests in ``tests/integration_tests.cpp``: the
same list of JSON inputs from ``data/integration_tests`` is driven through the
binding exactly the way the C++ ``wmtk_wrapper`` / ``app/main.cpp`` drives them
(parse JSON, inject ``json_input_file``, dispatch on ``application``).

For every input we check, like the C++ ``CHECK_NOTHROW``, that the binding runs
to completion without raising.

In addition we verify that the binding produces *the same result* as the native
C++ application (``wmtk_app``):

* **Serial inputs** (``num_threads`` <= 1) are deterministic, so the mesh files
  the binding writes are compared byte-for-byte against the files ``wmtk_app``
  writes for the same input. ``.log`` files are ignored (they only differ by
  timing).
* **Multithreaded inputs** (``num_threads`` > 1, e.g. ``tetwild_octocat``) use
  the parallel scheduler, whose ordering is non-deterministic; their output is
  *not* expected to be identical, so for those we only check termination.

The result comparison needs a ``wmtk_app`` binary. To keep the comparison
apples-to-apples it prefers the ``wmtk_app`` that scikit-build co-builds next to
the binding (``build/<config>/app/wmtk_app``), falling back to a top-level
``build/app/wmtk_app``; set ``WMTK_APP`` to override. If none is found the
comparison is skipped and only termination is checked.

Run with::

    build/venv/bin/python -m pytest tests/python -v
"""

import json
import os
import subprocess
import sys
import warnings
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]  # tests/python -> tests -> repo root

# Test data (same directory the C++ integration tests read from).
DATA_DIR = Path(
    os.environ.get("WMTK_DATA_DIR", REPO_ROOT / "data" / "integration_tests")
)

# Native C++ CLI used as the reference for the result comparison.
#
# The byte-for-byte comparison is only meaningful when the reference app and the
# binding are built from the same configuration: the iterative geometry code is
# sensitive enough that two independently-configured build trees (e.g. a stock
# `cmake build/` vs. the scikit-build `build/<config>/` tree the wheel is built
# in) can take divergent discrete decisions on the heaviest cases. So we prefer
# the `wmtk_app` that scikit-build co-builds alongside the binding under
# `build/<config>/app/`, and only fall back to a top-level `build/app/`.
# Override explicitly with WMTK_APP.


def _find_reference_app():
    env = os.environ.get("WMTK_APP")
    if env:
        return Path(env)
    exe = "wmtk_app.exe" if os.name == "nt" else "wmtk_app"
    candidates = [
        REPO_ROOT / "build" / "Release" / "app" / exe,
        REPO_ROOT / "build" / "RelWithDebInfo" / "app" / exe,
        REPO_ROOT / "build" / "Debug" / "app" / exe,
        REPO_ROOT / "build" / "app" / exe,
    ]
    for c in candidates:
        if c.exists():
            return c
    return candidates[-1]  # nominal default; if missing, comparison is skipped


APP = _find_reference_app()

# Subprocess helper that loads a JSON and calls wildmeshing.wildmeshing (see _run_binding.py).
RUN_BINDING = HERE / "_run_binding.py"

# Per-run wall-clock cap (seconds) so a hang fails loudly instead of stalling.
RUN_TIMEOUT = 1800

# Exactly the `input_files` list from tests/integration_tests.cpp.
# (image_simulation_energies_2d is commented out there as well.)
INPUT_FILES = [
    "image_simulation_remeshing_2d.json",
    # "image_simulation_energies_2d.json",
    "image_simulation_fill_holes_2d.json",
    "image_simulation_keep_lcc_2d.json",
    "image_simulation_tight_seal_2d.json",
    "image_simulation_replace_tags_2d.json",
    "image_simulation_resolve_intersections_2d.json",
    "image_simulation_tag_priority_2d.json",
    "image_simulation_double_sphere_3d.json",
    "image_simulation_double_sphere_notop_3d.json",
    "image_simulation_tight_seal_3d.json",
    "image_simulation_replace_tags_3d.json",
    "image_simulation_resolve_intersections_3d.json",
    "image_simulation_tag_priority_3d.json",
    "isotropic_remeshing_bunny.json",
    "isotropic_remeshing_double_sphere.json",
    "isotropic_remeshing_piece.json",
    "qslim_double_sphere.json",
    "qslim_octocat.json",
    "shortest_edge_collapse_101633.json",
    "shortest_edge_collapse_double_sphere.json",
    "shortest_edge_collapse_octocat.json",
    "shortest_edge_collapse_sphere_with_env.json",
    "tetwild_double_sphere.json",
    "tetwild_octocat.json",
    "tetwild_sphere.json",
    "topological_offset_2d.json",
    "topological_offset_2d_vertex_input.json",
    "topological_offset_3d_edge_input.json",
    "topological_offset_3d.json",
    "manifold_extraction_3d.json",
    "triwild_puzzle.json",
]

# Applications whose `output` default carries a ".obj" extension. The others
# (tetwild, image_simulation, topological_offset, ...) take an extension-less
# base name and append their own (".msh", "_surface.obj", "_<n>.vtu", ...).
_OBJ_OUTPUT_APPS = {"isotropic_remeshing", "qslim", "shortest_edge_collapse"}


def _load(json_file: Path) -> dict:
    with open(json_file) as f:
        return json.load(f)


def _effective_num_threads(j: dict) -> int:
    # `num_threads` defaults to 0 (serial) for every component in this list.
    # The scheduler only forks worker threads when num_threads > 0, and results
    # are only non-deterministic when there is more than one worker.
    return int(j.get("num_threads", 0))


def _output_base(app: str, out_dir: Path) -> str:
    name = "out.obj" if app in _OBJ_OUTPUT_APPS else "out"
    return str(out_dir / name)


def _is_result_file(p: Path) -> bool:
    """A file that represents an algorithm result (not a log)."""
    return p.is_file() and p.suffix.lower() != ".log" and p.name != "_run.log"


def _result_files(d: Path):
    return sorted(p.relative_to(d) for p in d.rglob("*") if _is_result_file(p))


def _run(cmd, cwd: Path, what: str):
    """Run a subprocess, surfacing captured output if it fails."""
    proc = subprocess.run(
        [str(c) for c in cmd],
        cwd=str(cwd),
        capture_output=True,
        text=True,
        timeout=RUN_TIMEOUT,
    )
    if proc.returncode != 0:
        tail = (proc.stdout + proc.stderr)[-4000:]
        raise AssertionError(
            f"{what} failed (exit {proc.returncode})\n"
            f"cmd: {' '.join(str(c) for c in cmd)}\n--- output tail ---\n{tail}"
        )
    return proc


def _run_binding(json_file: Path, app: str, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    _run(
        [sys.executable, RUN_BINDING, json_file, _output_base(app, out_dir)],
        cwd=out_dir,
        what=f"wildmeshing binding on {json_file.name}",
    )


def _run_app_reference(json_file: Path, app: str, out_dir: Path):
    """Run the native wmtk_app on the same input, writing into ``out_dir``.

    A temporary JSON is written next to the original (so the relative ``input``
    path still resolves) with ``output`` pointed at ``out_dir``.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    j = _load(json_file)
    j["output"] = _output_base(app, out_dir)
    tmp_json = DATA_DIR / f".pyitest_{json_file.stem}.json"
    tmp_json.write_text(json.dumps(j))
    try:
        _run(
            [APP, "-j", tmp_json],
            cwd=out_dir,
            what=f"wmtk_app on {json_file.name}",
        )
    finally:
        tmp_json.unlink(missing_ok=True)


@pytest.mark.parametrize("input_file", INPUT_FILES)
def test_integration(input_file, tmp_path):
    json_file = DATA_DIR / input_file
    assert json_file.exists(), f"missing integration input: {json_file}"

    j = _load(json_file)
    app = j["application"]
    nthreads = _effective_num_threads(j)

    # (1) Termination parity with the C++ CHECK_NOTHROW: the binding must run
    #     the whole application without raising / crashing.
    py_dir = tmp_path / "py"
    _run_binding(json_file, app, py_dir)

    if nthreads > 1:
        # Multithreaded -> non-deterministic ordering. Only termination is
        # checked; we still assert the run produced some output.
        assert _result_files(py_dir), (
            f"{input_file}: multithreaded run produced no output files"
        )
        return

    # (2) Result parity: serial runs are deterministic, so the binding's output
    #     must match the native C++ app byte-for-byte.
    #
    #     When the reference app is unavailable (e.g. when testing an installed
    #     wheel in CI, where only the binding is present) we cannot compare, but
    #     the binding has still run the whole application end-to-end on real
    #     data, so the test passes on termination. We just record that the
    #     byte-for-byte comparison was not performed.
    if not APP.exists():
        assert _result_files(py_dir), (
            f"{input_file}: run produced no output files"
        )
        warnings.warn(
            f"{input_file}: reference app not found at {APP}; "
            f"skipped C++/Python byte-for-byte comparison "
            f"(set WMTK_APP to enable it)",
            stacklevel=2,
        )
        return

    cpp_dir = tmp_path / "cpp"
    _run_app_reference(json_file, app, cpp_dir)

    cpp_files = _result_files(cpp_dir)
    py_files = _result_files(py_dir)
    assert cpp_files == py_files, (
        f"{input_file}: output file sets differ\n"
        f"  C++: {[str(p) for p in cpp_files]}\n"
        f"  py : {[str(p) for p in py_files]}"
    )
    assert cpp_files, f"{input_file}: no result files produced to compare"

    mismatched = [
        str(rel)
        for rel in cpp_files
        if (cpp_dir / rel).read_bytes() != (py_dir / rel).read_bytes()
    ]
    assert not mismatched, (
        f"{input_file}: binding output differs from C++ app for: {mismatched}"
    )
