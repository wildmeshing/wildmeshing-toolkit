"""Tier 2 — end-to-end Laplacian smoothing: fair a staircase tag_0/ambient
interface (2D) and verify roughness drops while the mesh stays valid. The case
runs once per engine ("python", the reference glue, and "cpp", its port in the
wmtk component polyfem_ops); the assertions are physical and hold for either.
Needs PolyFEM. This is also the 2D path of the polyfem pipeline."""
import numpy as np
import pytest

from simwild import simwild as wm

from conftest import ENGINES, needs_polyfem
from geo import (interface_polyline_2d, polyline_length, roughness_2d,
                 signed_volumes)

SEL = {"region": "tag_0", "filter": "ambient"}


@needs_polyfem
@pytest.mark.parametrize("engine", ENGINES)
def test_smoothing_reduces_interface_roughness(jagged2d, tmp_path, engine):
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
        engine=engine,
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
