"""Tier 2 — end-to-end Laplacian smoothing: fair a staircase tag_0/ambient
interface (2D) and verify roughness drops while the mesh stays valid. Needs
simwild built with polyfem (-DWMTK_WITH_POLYFEM=ON). This is also the 2D
path of the polyfem pipeline."""
import re

import h5py
import numpy as np
import pytest

from simwild import simwild as wm

from conftest import needs_polyfem, run_cpp
from geo import (interface_polyline_2d, polyline_length, roughness_2d,
                 signed_volumes)

SEL = {"region": "tag_0", "filter": "ambient"}


@needs_polyfem
def test_smoothing_reduces_interface_roughness(jagged2d, tmp_path):
    coords0, edges0 = interface_polyline_2d(jagged2d, SEL)
    rough0 = roughness_2d(coords0, edges0)
    len0 = polyline_length(coords0, edges0)
    assert rough0 > np.pi, "fixture should start visibly jagged"

    out_msh = tmp_path / "smoothed.msh"
    wm.laplacian_smoothing(
        mesh=str(jagged2d),
        interfaces=[SEL],
        output=str(tmp_path / "smoothed"),
        others={"use_fitting": True, "use_laplacian": True,
                "weight_laplacian": 1e3, "normalize_penalties": True,
                "scale": 1e-3},
    )

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


@needs_polyfem
def test_per_interface_weight_scales_the_laplacian_rows(jagged2d, tmp_path):
    """`weight` on a selection replaces weight_laplacian for that interface's
    nodes. This fixture has ONE selection, so every interface node is its node
    and the whole penalty matrix is scaled by sqrt(4 W / W) = 2."""
    def laplacian_values(name, **params):
        run_cpp(jagged2d, "laplacian_smoothing", tmp_path / name,
                weight_laplacian=1e3, **params)
        path = (tmp_path / name / "smooth_input"
                / "interface_constraint_laplacian.hdf5")
        with h5py.File(path) as f:
            return f["A_triplets/values"][()], f["b"][()]

    a0, b0 = laplacian_values("plain", interfaces=[SEL])
    a1, b1 = laplacian_values("weighted", interfaces=[{**SEL, "weight": 4e3}])

    # Doubling is exact in binary floating point, so this is an equality.
    assert np.array_equal(a1, 2.0 * a0)
    assert np.array_equal(b1, 2.0 * b0)

    # minimum_separation's sides do not take a weight: the key belongs to the
    # smoothing spec, and jse names the rule that refused the selection.
    with pytest.raises(RuntimeError, match=re.escape('"/collision_pairs/*/*"')):
        run_cpp(jagged2d, "minimum_separation", tmp_path / "rejected", sep=1e-3,
                collision_pairs=[[{**SEL, "weight": 4e3}, "ambient"]])
