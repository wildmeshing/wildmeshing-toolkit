"""Tier 2 — end-to-end Laplacian smoothing: fair a staircase tag_0/ambient
interface (2D) and verify roughness drops while the mesh stays valid; and the
same smoothing in process (smooth) against PolyFEM_bin. Needs PolyFEM."""
import numpy as np
import pytest

import wildmeshing as wm
from wildmeshing.polyfem_ops.laplacian_smoothing import laplacian_smoothing, smooth

from conftest import needs_in_process, needs_polyfem
from geo import (interface_polyline_2d, polyline_length, roughness_2d,
                 signed_volumes)

SEL = {"region": "tag_0", "filter": "ambient"}
OPTIONS = {
    "interfaces": [SEL],
    "use_fitting": True,
    "use_laplacian": True,
    "weight_laplacian": 1e3,
    "normalize_penalties": True,
    "scale": 1e-3,
}


@needs_polyfem
def test_smoothing_reduces_interface_roughness(jagged2d, tmp_path):
    coords0, edges0 = interface_polyline_2d(jagged2d, SEL)
    rough0 = roughness_2d(coords0, edges0)
    len0 = polyline_length(coords0, edges0)
    assert rough0 > np.pi, "fixture should start visibly jagged"

    out_msh = tmp_path / "smooth" / "smoothed.msh"
    laplacian_smoothing(dict(OPTIONS, input=str(jagged2d), output=str(out_msh.with_suffix(""))))

    assert out_msh.exists()

    coords1, edges1 = interface_polyline_2d(out_msh, SEL)
    rough1 = roughness_2d(coords1, edges1)
    len1 = polyline_length(coords1, edges1)

    # 1. The staircase got measurably straighter and shorter.
    assert rough1 < 0.7 * rough0, f"roughness {rough0:.3f} -> {rough1:.3f}"
    assert len1 < len0

    # 2. Nothing inverted.
    a0 = signed_volumes(jagged2d)
    a1 = signed_volumes(out_msh)
    assert np.all(np.sign(a1) == np.sign(a0))

    # 3. Fitting kept the interface anchored: same topology, bounded motion.
    assert edges1.shape == edges0.shape
    moved = np.linalg.norm(coords1 - coords0, axis=1)
    assert moved.max() < 2.0, f"max node displacement {moved.max():.2f}"


@needs_in_process
@needs_polyfem
@pytest.mark.parametrize("fixture", ["jagged2d", "boxes3d"])
def test_smooth_in_process_matches_polyfem_bin(fixture, request, tmp_path, monkeypatch):
    """smooth() on the mesh dictionary moves the vertices as laplacian_smoothing()
    moves them through PolyFEM_bin, and writes no files."""
    msh = request.getfixturevalue(fixture)
    out_msh = tmp_path / "file" / "smoothed.msh"
    laplacian_smoothing(dict(OPTIONS, input=str(msh), output=str(out_msh.with_suffix(""))))

    memory = tmp_path / "memory"
    memory.mkdir()
    monkeypatch.chdir(memory)
    mesh = wm.read_mesh(msh)
    meshes, report = smooth(dict(OPTIONS, input="m"), {"m": mesh})
    assert list(memory.iterdir()) == []

    smoothed = meshes["mesh"]
    file_V = wm.read_mesh(out_msh)["V"]
    assert report["max_displacement"] > 0.1      # mesh units: it did smooth
    np.testing.assert_allclose(smoothed["V"], file_V, rtol=0, atol=1e-9)
    for cell, block in mesh["cells"].items():    # all but the vertices as they were
        np.testing.assert_array_equal(smoothed["cells"][cell]["connectivity"], block["connectivity"])
