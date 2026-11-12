"""PolyFEM in process: wildmeshing._core.polyfem_simulate and polyfem_sim.simulate. Needs a
wildmeshing built with WMTK_WITH_POLYFEM; the comparison with PolyFEM_bin needs POLYFEM_BIN too."""
import json

import numpy as np
import pytest

import wildmeshing as wm
from wildmeshing.polyfem_ops import polyfem_sim
from wildmeshing.polyfem_ops.polyfem_utils import mesh_group_resources
from wildmeshing.polyfem_ops.polyfem_sim.msh_boundary_extractor import simulation_mesh

from conftest import needs_in_process, needs_polyfem

SKIN_0 = {"region": "tag_0", "filter": "ambient"}


def _rigid(dim):
    """A translation plus an infinitesimal rotation, u = b + W x: no strain, so it is the exact
    solution of a linear elastic body whose boundary it moves, whatever the mesh."""
    b = np.array([0.1, 0.2, 0.3])[:dim]
    W = np.zeros((dim, dim))
    W[0, 1], W[1, 0] = 0.01, -0.01
    value = [f"{b[0]} + 0.01*y", f"{b[1]} - 0.01*x"] + ([f"{b[2]}"] if dim == 3 else [])
    return b, W, value


@needs_in_process
@pytest.mark.parametrize("dim", [3, 2])
def test_displacement_is_in_vertex_order(boxes3d, jagged2d, dim):
    sim = simulation_mesh(wm.read_mesh(boxes3d if dim == 3 else jagged2d), ["tag_0"], [SKIN_0])
    group, mesh_dim, name_to_tag, selections = polyfem_sim.polyfem_mesh(sim)
    assert mesh_dim == dim and group["vertices"].shape[1] == dim
    b, W, value = _rigid(dim)
    args = {
        "geometry": [{"mesh": "m"}],
        "space": {"discr_order": 1},
        "materials": [{"id": name_to_tag["tag_0"], "type": "LinearElasticity", "E": 1e3, "nu": 0.3}],
        "boundary_conditions": {
            "dirichlet_boundary": [{"id": selections[("tag_0", "ambient")], "value": value}]},
        "output": {"advanced": {"save_time_sequence": False}},
    }
    u = wm._core.polyfem_simulate(args, *mesh_group_resources("m", group))["displacement"]
    np.testing.assert_allclose(u, b + group["vertices"] @ W.T, rtol=0, atol=1e-9)


def _two_bodies_cfg(output=None):
    """tag_0 held by its skin, tag_1 falling: body ids (materials differ) and boundary ids (the
    held skin) both change the result if they are wrong."""
    cfg = {
        "materials": {
            "tag_0": {"type": "NeoHookean", "E": 1e7, "nu": 0.3, "rho": 1000},
            "tag_1": {"type": "NeoHookean", "E": 1e6, "nu": 0.4, "rho": 1000},
        },
        "boundary_conditions": {
            "rhs": [0, 0, 9.81],
            "dirichlet_boundary": [{"selection": SKIN_0, "value": [0, 0, 0]}],
        },
        "time": {"tend": 0.03, "dt": 0.01},
        "save_vtu": False,
        "save_restart": False,
    }
    if output is not None:
        cfg["output"] = output
    return cfg


@needs_in_process
def test_simulate_writes_no_files(boxes3d, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)  # where PolyFEM's relative outputs would land
    sim = simulation_mesh(wm.read_mesh(boxes3d), ["tag_0", "tag_1"], [SKIN_0])
    before = sorted(tmp_path.rglob("*"))
    steps = []
    u = polyfem_sim.simulate(_two_bodies_cfg(), sim,
                             step_callback=lambda step, d: steps.append((step, d.copy())))
    assert sorted(tmp_path.rglob("*")) == before
    assert [step for step, _ in steps] == [0, 1, 2, 3]
    np.testing.assert_array_equal(steps[-1][1], u)
    assert u.shape == sim["V"].shape and np.abs(u).max() > 0


@needs_in_process
def test_simulate_without_out_dir_refuses_files(boxes3d):
    sim = simulation_mesh(wm.read_mesh(boxes3d), ["tag_0"], [SKIN_0])
    with pytest.raises(ValueError, match="save_vtu writes files"):
        polyfem_sim.simulate(dict(_two_bodies_cfg(), save_vtu=True), sim)


@needs_in_process
@needs_polyfem
def test_simulate_matches_polyfem_bin(boxes3d, tmp_path):
    """The same simulation from the mesh file through PolyFEM_bin (run) and in process
    (simulate): the same JSON but for the mesh, the same solution, and simulate's displacement is
    PolyFEM's solution in input vertex order (reorder_nodes)."""
    sim = simulation_mesh(wm.read_mesh(boxes3d), ["tag_0", "tag_1"], [SKIN_0])
    msh = tmp_path / "sim.msh"
    wm.write_mesh(msh, sim)
    output = {"data": {"solution": "solution.txt", "advanced": {"reorder_nodes": True}}}

    polyfem_sim.run(dict(_two_bodies_cfg(output), input_msh=str(msh)), out_dir=tmp_path / "file")
    u = polyfem_sim.simulate(_two_bodies_cfg(output), sim, out_dir=tmp_path / "memory")

    file_json = json.loads((tmp_path / "file" / "sim_input" / "simulation.json").read_text())
    assert file_json["geometry"][0]["mesh"] == str(msh.resolve())
    file_u = np.loadtxt(tmp_path / "file" / "sim_output" / "solution.txt")
    memory_u = np.loadtxt(tmp_path / "memory" / "sim_output" / "solution.txt")
    np.testing.assert_allclose(memory_u, file_u, rtol=1e-9, atol=1e-12)
    np.testing.assert_allclose(u, memory_u, rtol=1e-12, atol=1e-15)
