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
@pytest.mark.parametrize("strategy,extra", [
    ("dhat", {}),
    ("stiffness", {}),
])
def test_minimum_separation_reaches_target(boxes3d, tmp_path, strategy, extra):
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
        "strategy": strategy,
        "output_msh": str(out_msh),
        **extra,
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
    if strategy == "stiffness":
        # sep is a hard floor and dhat = sep*(1+rtol) caps the overshoot
        # structurally (contact force vanishes beyond dhat).
        assert gap <= target_mesh_units * (1.0 + cfg["rtol"]) * 1.02, (
            f"gap {gap:.4f} exceeds the structural bound")

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


@needs_polyfem
def test_minimum_separation_protected_region(boxes3d, tmp_path):
    """protected_regions: tag_1 is hard-pinned — tag_0 does all the moving
    and every node of tag_1's cells stays exactly at rest."""
    from simwild.polyfem_ops.mesh_core import TaggedMesh, select_region_nodes

    out_msh = tmp_path / "separated.msh"
    cfg = {
        "input_msh": str(boxes3d),
        "collision_pairs": [[SEL_A, SEL_B]],
        "sep": SEP,
        "scale": SCALE,
        "rtol": 1e-1,
        "max_iterations": 8,
        "strategy": "stiffness",
        "protected_regions": ["tag_1"],
        "output_msh": str(out_msh),
    }
    before = TaggedMesh(str(boxes3d))
    pin_ids = select_region_nodes(before, ["tag_1"])

    ms.run(cfg, out_dir=tmp_path / "sep")

    gap = min_separation_3d(out_msh, SEL_A, SEL_B)
    target_mesh_units = SEP / SCALE
    assert gap >= target_mesh_units * (1.0 - cfg["rtol"] - 0.05), (
        f"achieved gap {gap:.4f} < target {target_mesh_units:.4f}")

    after = TaggedMesh(str(out_msh))
    drift = np.abs(after.coords[pin_ids] - before.coords[pin_ids]).max()
    assert drift < 1e-9, f"protected tag_1 nodes moved by {drift:.3e}"
    # and the unprotected side really did the moving
    moved_ids = select_region_nodes(before, ["tag_0"])
    moved = np.abs(after.coords[moved_ids] - before.coords[moved_ids]).max()
    assert moved > 0.1, "tag_0 did not move"
