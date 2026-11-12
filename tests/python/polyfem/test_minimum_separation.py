"""Tier 2 — minimum separation, in process: two interior boxes with a 1-unit
gap pushed to a target separation (minimum_separation on the .msh, separate on
the mesh dictionary); the first solve against PolyFEM_bin; the warm start.
Needs a wildmeshing built with WMTK_WITH_POLYFEM; PolyFEM_bin for the
comparison."""
import copy

import numpy as np
import pytest

import wildmeshing as wm
from wildmeshing.polyfem_ops import minimum_separation as ms
from wildmeshing.polyfem_ops import spec
from wildmeshing.polyfem_ops.constraints import write_pin_constraint_hdf5
from wildmeshing.polyfem_ops.mesh_core import TaggedMesh, select_region_nodes
from wildmeshing.polyfem_ops.polyfem_utils import (OPT_DEFAULTS, build_polyfem_json,
                                                   get_mesh_info, polyfem_bin,
                                                   step_make_interface_constraint,
                                                   step_run_polyfem_single,
                                                   step_write_deformed_msh,
                                                   _write_polyfem_reduced_msh)

from conftest import needs_in_process, needs_polyfem
from geo import min_separation_3d, signed_volumes

SEL_A = {"region": "tag_0", "filter": "ambient"}
SEL_B = {"region": "tag_1", "filter": "ambient"}
SCALE = 1e-3            # mesh units -> solver units
GAP0 = 1.0              # built into the fixture
SEP = 1.5e-3            # solver units: target gap of 1.5 mesh units
OPTIONS = {"collision_pairs": [[SEL_A, SEL_B]], "sep": SEP, "scale": SCALE, "rtol": 1e-1}
DHAT = 1.9 * GAP0 * SCALE   # the dhat ramp's first solve after its probe (dhat_growth * gap)


@needs_in_process
@pytest.mark.parametrize("strategy", ["dhat", "stiffness"])
def test_minimum_separation_reaches_target(boxes3d, tmp_path, strategy):
    out = tmp_path / "sep" / "separated"
    report = ms.minimum_separation(dict(
        OPTIONS, input=str(boxes3d), output=str(out), max_iterations=6, strategy=strategy))
    out_msh = out.with_suffix(".msh")
    assert sorted(p.name for p in out_msh.parent.iterdir()) == ["separated.msh"]

    # 1. The bodies separated to the target, as the geometry and the loop measure it.
    gap = min_separation_3d(out_msh, SEL_A, SEL_B)
    target_mesh_units = SEP / SCALE
    assert report["gap"] >= SEP
    assert gap >= target_mesh_units * (1.0 - OPTIONS["rtol"] - 0.05), (
        f"achieved gap {gap:.4f} < target {target_mesh_units:.4f}")
    if strategy == "stiffness":
        # sep is a hard floor and dhat = sep*(1+rtol) caps the overshoot
        # structurally (contact force vanishes beyond dhat).
        assert gap <= target_mesh_units * (1.0 + OPTIONS["rtol"]) * 1.02, (
            f"gap {gap:.4f} exceeds the structural bound")

    # 2. No needless motion: no vertex moved further than the gap grew, which moving one
    # body rigidly the whole way would take.
    V0, V1 = wm.read_mesh(boxes3d)["V"], wm.read_mesh(out_msh)["V"]
    moved = np.linalg.norm(V1 - V0, axis=1).max()
    assert moved <= gap - GAP0, f"a vertex moved {moved:.4f}, the gap grew {gap - GAP0:.4f}"

    # 3. No element inverted (signs of signed volumes preserved).
    vol_before, vol_after = signed_volumes(boxes3d), signed_volumes(out_msh)
    assert vol_after.shape == vol_before.shape
    assert np.all(np.sign(vol_after) == np.sign(vol_before))

    # 4. The full tag set survived onto the deformed mesh.
    import gmsh
    gmsh.initialize()
    try:
        gmsh.open(str(out_msh))
        names = {gmsh.model.getPhysicalName(d, t)
                 for d, t in gmsh.model.getPhysicalGroups(3)}
    finally:
        gmsh.finalize()
    assert names == {"ambient", "tag_0", "tag_1"}


@needs_in_process
def test_separate_protected_region(boxes3d, tmp_path, monkeypatch):
    """protected_regions: tag_1 is hard-pinned — tag_0 does all the moving
    and every node of tag_1's cells stays at rest; separate writes no files."""
    monkeypatch.chdir(tmp_path)   # where PolyFEM's relative outputs would land
    mesh = wm.read_mesh(boxes3d)
    before = sorted(tmp_path.rglob("*"))
    meshes, report = ms.separate(dict(
        OPTIONS, input="m", max_iterations=8, strategy="stiffness",
        protected_regions=["tag_1"]), {"m": mesh})
    assert sorted(tmp_path.rglob("*")) == before
    assert report["gap"] >= SEP

    tagged = TaggedMesh.from_mesh(mesh)
    V0, V1 = mesh["V"], meshes["mesh"]["V"]
    pinned = select_region_nodes(tagged, ["tag_1"])
    drift = np.abs(V1[pinned] - V0[pinned]).max()
    assert drift < 1e-9, f"protected tag_1 nodes moved by {drift:.3e}"
    free = select_region_nodes(tagged, ["tag_0"])
    assert np.abs(V1[free] - V0[free]).max() > 0.1, "tag_0 did not move"


def _rounded(x):
    """x with every number at 12 significant digits."""
    if isinstance(x, dict):
        return {k: _rounded(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_rounded(v) for v in x]
    if isinstance(x, float):
        return float(f"{x:.12g}")
    return x


def _strip_paths(j):
    """The separation JSON without what names its inputs and outputs, its numbers to
    rounding: the AMIPS weights divide by each tag's volume, which the file route sums from
    the .msh (get_mesh_info) and the memory route from the arrays (reduced_mesh)."""
    j = copy.deepcopy(j)
    j["geometry"][0].pop("mesh")
    for c in j.get("constraints", {}).get("soft", []):
        c.pop("data")
    j.get("constraints", {}).pop("hard", None)
    for k in ("mesh", "linear_map", "collision_body_ids"):
        j["contact"]["collision_mesh"].pop(k)
    j.pop("output")
    return _rounded(j)


@needs_in_process
@needs_polyfem
@pytest.mark.parametrize("protected", [[], ["tag_1"]])
def test_first_solve_matches_polyfem_bin(boxes3d, tmp_path, protected):
    """The loop's first contact solve, which starts from rest, in process and through
    PolyFEM_bin from the files the old file route wrote: the same JSON but for the names of
    its inputs, and the same deformed mesh. With protected_regions, the hard constraints too."""
    p = spec.validate(spec.load_spec("minimum_separation"),
                      dict(OPTIONS, input="m", protected_regions=protected))
    mesh = wm.read_mesh(boxes3d)
    cfg, memory_json, resources, attributes, _ = ms._inputs(p, mesh)

    d = tmp_path / "file"
    d.mkdir()
    sides, _ = ms._normalize_collision_pairs(p["collision_pairs"])
    step_make_interface_constraint(
        str(boxes3d), d, cfg["useGraphLaplacian"], cfg["normalizePenalties"], SCALE,
        selections=sides,
        smooth_positions=OPT_DEFAULTS["smoothDisplacementsOrPositions"] == 1)
    reduced = d / "reduced.msh"
    _write_polyfem_reduced_msh(str(boxes3d), str(reduced), ambient_like_tags=[])
    material_tags, dim, name_to_tag, tag_to_count, tag_to_volume = get_mesh_info(str(reduced))
    solution = d / "solution.txt"
    file_json = build_polyfem_json(
        cfg, reduced, d, material_tags, dim, solution, name_to_tag=name_to_tag,
        tag_to_count=tag_to_count, tag_to_volume=tag_to_volume)
    if protected:
        pin = d / "protected_pin.hdf5"
        write_pin_constraint_hdf5(
            str(pin), select_region_nodes(TaggedMesh(str(boxes3d)), protected), dim)
        file_json.setdefault("constraints", {})["hard"] = [str(pin)]
        assert memory_json["constraints"]["hard"] == ["protected_pin.hdf5"]
    for j in (file_json, memory_json):
        j["contact"]["dhat"] = DHAT
    assert _strip_paths(file_json) == _strip_paths(memory_json)

    step_run_polyfem_single(polyfem_bin(), file_json, d / "separation.json", d / "out")
    step_write_deformed_msh(boxes3d, solution, d / "deformed.msh", SCALE)
    u = wm._core.polyfem_simulate(memory_json, resources, attributes)["displacement"]
    V = mesh["V"] + u / SCALE
    file_V = wm.read_mesh(d / "deformed.msh")["V"]
    assert np.abs(file_V - mesh["V"]).max() > 0.1     # mesh units: the contact moved it
    np.testing.assert_allclose(V, file_V, rtol=0, atol=1e-9)


@needs_in_process
def test_warm_start_starts_at_the_solution(boxes3d):
    """A solve started from an earlier solve's solution starts where that one ended: one
    Newton iteration from it gives that displacement; one from rest does not."""
    p = spec.validate(spec.load_spec("minimum_separation"), dict(OPTIONS, input="m"))
    _, j, resources, attributes, _ = ms._inputs(p, wm.read_mesh(boxes3d))
    j["contact"]["dhat"] = DHAT
    converged = wm._core.polyfem_simulate(j, resources, attributes)
    u = converged["displacement"]

    one = copy.deepcopy(j)
    one["solver"]["nonlinear"]["max_iterations"] = 1
    warm = wm._core.polyfem_simulate(one, resources, attributes,
                                     initial_solution=converged["solution"])["displacement"]
    cold = wm._core.polyfem_simulate(one, resources, attributes)["displacement"]
    np.testing.assert_allclose(warm, u, rtol=0, atol=1e-9 * np.abs(u).max())
    assert np.abs(cold - u).max() > 0.1 * np.abs(u).max()
