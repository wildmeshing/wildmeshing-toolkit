"""Tier 2 — end-to-end Laplacian smoothing: fair a staircase tag_0/ambient
interface (2D) and verify roughness drops while the mesh stays valid.
Needs PolyFEM. Also exercises the 2D pipeline path of minimum_separation."""
import numpy as np
import pytest

from simwild.polyfem_ops import minimum_separation as ms
from simwild.polyfem_ops.laplacian_smoothing import run as ls_run

from conftest import needs_polyfem
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
    ls_run({
        "input_msh": str(jagged2d),
        "interfaces": [SEL],
        "useFitting": True,
        "useLaplacian": True,
        "weight_laplacian": 1e3,
        "normalizePenalties": True,
        "scale": 1e-3,
        "output_msh": str(out_msh),
    }, out_dir=tmp_path / "smooth")

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
