"""Shared pytest setup for the Python integration tests."""

import os
from pathlib import Path

import pytest

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
