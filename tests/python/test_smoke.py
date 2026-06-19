"""Fast smoke test for the pywmtk bindings (no test data required).

This is what the wheel CI runs against every built wheel (see the
``[tool.cibuildwheel]`` ``test-command`` in ``pyproject.toml``): it confirms the
compiled extension imports on the target platform/Python and that the
``wmtk`` dispatch entry point is wired up and executes its error paths.
"""

import pytest

import pywmtk


def test_module_has_entry_point():
    assert hasattr(pywmtk, "wmtk")
    assert callable(pywmtk.wmtk)


def test_missing_application_raises():
    # Mirrors the C++ guard: input must contain an `application` key.
    with pytest.raises(Exception):
        pywmtk.wmtk({})


def test_unknown_application_raises():
    with pytest.raises(Exception):
        pywmtk.wmtk({"application": "this_application_does_not_exist"})
