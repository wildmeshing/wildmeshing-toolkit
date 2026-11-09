"""Shared pytest setup for the Python tests."""

import importlib.util
import os
from pathlib import Path

import pytest

# When wildmeshing is not pip-installed, fall back to the toolkit build tree,
# which mirrors the package in build/bin/wildmeshing.
try:
    import wildmeshing  # noqa: F401
except ImportError:
    import sys
    for up in Path(__file__).resolve().parents:
        cand = up / "build" / "bin"
        if (cand / "wildmeshing" / "__init__.py").is_file():
            sys.path.insert(0, str(cand))
            break

# The PolyFEM operations' tests need the polyfem extras: without them, polyfem/ is not collected.
if any(importlib.util.find_spec(m) is None for m in ["gmsh", "scipy", "h5py"]):
    collect_ignore = ["polyfem"]

_HERE = Path(__file__).resolve().parent
_REPO_ROOT = _HERE.parents[1]
DATA_DIR = Path(
    os.environ.get("WMTK_DATA_DIR", _REPO_ROOT / "data" / "integration_tests")
)


def _cleanup_temp_jsons():
    # Temporary JSONs written next to the originals when invoking wmtk_app
    # (see _run_app_reference in test_integration.py). Remove any that a
    # crashed run may have left behind.
    if DATA_DIR.exists():
        for stray in DATA_DIR.glob(".pyitest_*.json"):
            stray.unlink(missing_ok=True)


@pytest.fixture(scope="session", autouse=True)
def _temp_json_cleanup():
    _cleanup_temp_jsons()
    yield
    _cleanup_temp_jsons()
