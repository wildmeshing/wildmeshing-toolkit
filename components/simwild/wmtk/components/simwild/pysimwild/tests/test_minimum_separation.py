"""Tier 2 — end-to-end minimum separation: run ms.run on two interior boxes
with a 1-unit gap and verify the achieved separation. Needs PolyFEM."""
import numpy as np
import pytest

from simwild.polyfem_ops import minimum_separation as ms

from conftest import needs_polyfem
from geo import min_separation_3d, signed_volumes

SEL_A = {"region": "tag_0", "filter": "ambient"}
SEL_B = {"region": "tag_1", "filter": "ambient"}
SCALE = 1e-3            # mesh units -> solver units
GAP0 = 1.0              # built into the fixture
SEP = 1.5e-3            # solver units: target gap of 1.5 mesh units


@needs_polyfem
def test_minimum_separation_reaches_target(boxes3d, tmp_path):
    out_msh = tmp_path / "separated.msh"
    cfg = {
        "input_msh": str(boxes3d),
        "collision_pairs": [[SEL_A, SEL_B]],
        "sep": SEP,
        "scale": SCALE,
        "useFitting": True,
        "useLaplacian": True,
        "normalizePenalties": True,
        "rtol": 1e-1,
        "max_iterations": 6,
        "output_msh": str(out_msh),
    }
    vol_before = signed_volumes(boxes3d)

    ms.run(cfg, out_dir=tmp_path / "sep")

    assert out_msh.exists()

    # 1. The bodies actually separated to (at least close to) the target.
    gap = min_separation_3d(out_msh, SEL_A, SEL_B)
    target_mesh_units = SEP / SCALE
    assert gap > GAP0 * 1.05, "bodies did not move apart"
    assert gap >= target_mesh_units * (1.0 - cfg["rtol"] - 0.05), (
        f"achieved gap {gap:.4f} < target {target_mesh_units:.4f}")

    # 2. No element inverted (signs of signed volumes preserved).
    vol_after = signed_volumes(out_msh)
    assert vol_after.shape == vol_before.shape
    assert np.all(np.sign(vol_after) == np.sign(vol_before))

    # 3. The full tag set survived onto the deformed mesh.
    import gmsh
    gmsh.initialize()
    try:
        gmsh.open(str(out_msh))
        names = {gmsh.model.getPhysicalName(d, t)
                 for d, t in gmsh.model.getPhysicalGroups(3)}
    finally:
        gmsh.finalize()
    assert names == {"ambient", "tag_0", "tag_1"}
