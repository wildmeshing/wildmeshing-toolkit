"""Tier 2 — end-to-end minimum separation: run the wrapper on two interior
boxes with a 1-unit gap and verify the achieved separation. Every case runs
once per engine ("python", the reference glue, and "cpp", its port in the wmtk
component polyfem_ops): the assertions below are physical, so they hold for
either implementation. Needs PolyFEM."""
import numpy as np
import pytest

from simwild import simwild as wm

from conftest import ENGINES
from geo import min_separation_3d, region_node_coords, signed_volumes

SEL_A = {"region": "tag_0", "filter": "ambient"}
SEL_B = {"region": "tag_1", "filter": "ambient"}
SCALE = 1e-3            # mesh units -> solver units
GAP0 = 1.0              # built into the fixture
SEP = 1.5e-3            # solver units: target gap of 1.5 mesh units
RTOL = 1e-1             # relative tolerance on the achieved separation


@pytest.mark.parametrize("engine", ENGINES)
@pytest.mark.parametrize("strategy", ["dhat", "stiffness"])
def test_minimum_separation_reaches_target(boxes3d, tmp_path, strategy, engine):
    out_msh = tmp_path / "separated.msh"
    vol_before = signed_volumes(boxes3d)

    wm.minimum_separation(
        mesh=str(boxes3d),
        collision_pairs=[[SEL_A, SEL_B]],
        sep=SEP,
        output=str(tmp_path / "separated"),
        others={"scale": SCALE, "use_fitting": True, "use_laplacian": True,
                "normalize_penalties": True, "rtol": RTOL,
                "max_iterations": 6, "strategy": strategy},
        engine=engine,
    )

    assert out_msh.exists()

    # 1. The bodies actually separated to (at least close to) the target.
    gap = min_separation_3d(out_msh, SEL_A, SEL_B)
    target_mesh_units = SEP / SCALE
    assert gap > GAP0 * 1.05, "bodies did not move apart"
    assert gap >= target_mesh_units * (1.0 - RTOL - 0.05), (
        f"achieved gap {gap:.4f} < target {target_mesh_units:.4f}")
    if strategy == "stiffness":
        # sep is a hard floor and dhat = sep*(1+rtol) caps the overshoot
        # structurally (contact force vanishes beyond dhat).
        assert gap <= target_mesh_units * (1.0 + RTOL) * 1.02, (
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


@pytest.mark.parametrize("engine", ENGINES)
def test_minimum_separation_protected_region(boxes3d, tmp_path, engine):
    """protected_regions: tag_1 is hard-pinned — tag_0 does all the moving
    and every node of tag_1's cells stays exactly at rest."""
    out_msh = tmp_path / "separated.msh"

    wm.minimum_separation(
        mesh=str(boxes3d),
        collision_pairs=[[SEL_A, SEL_B]],
        sep=SEP,
        output=str(tmp_path / "separated"),
        others={"scale": SCALE, "rtol": RTOL, "max_iterations": 8,
                "strategy": "stiffness", "protected_regions": ["tag_1"]},
        engine=engine,
    )

    gap = min_separation_3d(out_msh, SEL_A, SEL_B)
    target_mesh_units = SEP / SCALE
    assert gap >= target_mesh_units * (1.0 - RTOL - 0.05), (
        f"achieved gap {gap:.4f} < target {target_mesh_units:.4f}")

    # The write-back keeps every node tag, so a node is the same node in both
    # files.
    def displacement(region):
        before = region_node_coords(boxes3d, region)
        after = region_node_coords(out_msh, region)
        assert after.keys() == before.keys()
        return max(np.abs(after[t] - before[t]).max() for t in before)

    drift = displacement("tag_1")
    assert drift < 1e-9, f"protected tag_1 nodes moved by {drift:.3e}"
    # and the unprotected side really did the moving
    moved = displacement("tag_0")
    assert moved > 0.1, "tag_0 did not move"
