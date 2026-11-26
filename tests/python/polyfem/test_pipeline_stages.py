"""The PolyFEM operations as pipeline stages (wildmeshing.pipeline), in process: a stage gives
what its memory interface gives on the stage's input, in a process of its own too; a simulation
keeps PolyFEM's frames, which come back from the cache. Nothing is written but the cache. Needs a
wildmeshing built with WMTK_WITH_POLYFEM."""
from pathlib import Path

import numpy as np

import wildmeshing as wm
from wildmeshing.pipeline import Pipeline
from wildmeshing.polyfem_ops import polyfem_sim
from wildmeshing.polyfem_ops.laplacian_smoothing import smooth
from wildmeshing.polyfem_ops.polyfem_sim.msh_boundary_extractor import simulation_mesh

from conftest import needs_in_process

SKIN_0 = {"region": "tag_0", "filter": "ambient"}
SMOOTHING = {"interfaces": [SKIN_0], "use_fitting": True, "use_laplacian": True,
             "weight_laplacian": 1e3, "normalize_penalties": True, "scale": 1e-3}
# test_simulate's two bodies: tag_0 held by its skin, tag_1 falling; four steps, every other kept.
SIMULATION = {
    "bodies": ["tag_0", "tag_1"],
    "materials": {"tag_0": {"type": "NeoHookean", "E": 1e7, "nu": 0.3, "rho": 1000},
                  "tag_1": {"type": "NeoHookean", "E": 1e6, "nu": 0.4, "rho": 1000}},
    "boundary_conditions": {"rhs": [0, 0, 9.81],
                            "dirichlet_boundary": [{"selection": SKIN_0, "value": [0, 0, 0]}]},
    "time": {"tend": 0.04, "dt": 0.01},
    "output": {"paraview": {"skip_frame": 2}},
}


def stage(name, component, options):
    return {"name": name, "component": component,
            "options": dict(options, input={"from": "m", "output": "mesh"})}


def files(folder, cache):
    return sorted(p for p in folder.rglob("*") if cache not in p.parents and p != cache)


@needs_in_process
def test_a_smoothing_stage_smooths_its_input_in_a_process_of_its_own(jagged2d, tmp_path):
    p = Pipeline([stage("s", "laplacian_smoothing", SMOOTHING)], str(tmp_path),
                 inputs={"m": jagged2d.name})
    p.process = True
    before = files(tmp_path, Path(p.cache))
    smoothed = p.run()["s"]["outputs"]["mesh"]
    assert files(tmp_path, Path(p.cache)) == before
    direct = smooth(dict(SMOOTHING, input="m"), {"m": wm.read_mesh(jagged2d)})[0]["mesh"]
    np.testing.assert_allclose(smoothed["V"], direct["V"], rtol=0, atol=1e-12)
    assert np.abs(smoothed["V"] - wm.read_mesh(jagged2d)["V"]).max() > 0.1  # it did smooth


@needs_in_process
def test_a_minimum_separation_stage_reaches_its_target(boxes3d, tmp_path):
    sel_b = {"region": "tag_1", "filter": "ambient"}
    options = {"collision_pairs": [[SKIN_0, sel_b]], "sep": 1.5e-3, "scale": 1e-3, "rtol": 1e-1}
    r = Pipeline([stage("ms", "minimum_separation", options)], str(tmp_path),
                 inputs={"m": boxes3d.name}).run()["ms"]
    assert r["report"]["gap"] >= 1.5e-3 and r["report"]["solves"] >= 1


@needs_in_process
def test_a_simulation_stage_keeps_polyfem_s_frames_and_they_come_back_from_the_cache(
        boxes3d, tmp_path):
    p = Pipeline([stage("sim", "polyfem_sim", SIMULATION)], str(tmp_path),
                 inputs={"m": boxes3d.name})
    p.process = True  # the frames over the pipe
    before = files(tmp_path, Path(p.cache))
    r = p.run()["sim"]
    assert files(tmp_path, Path(p.cache)) == before
    assert r["report"]["steps"] == [0, 2, 4]
    np.testing.assert_allclose(r["report"]["time"], [0.0, 0.02, 0.04], rtol=0, atol=1e-15)

    # The frames are simulate()'s displacements at the kept steps, in mesh units: as close as
    # two runs of simulate() are -- 1.7e-9 mesh units apart at step 4 on the mac-mini (11-23),
    # PolyFEM's own solves not being bit for bit the same -- where a wrong scale, row order or
    # step is off by order 1.
    sim = simulation_mesh(wm.read_mesh(boxes3d), ["tag_0", "tag_1"], [SKIN_0])
    steps = {}
    cfg = {k: v for k, v in SIMULATION.items() if k != "bodies"}
    polyfem_sim.simulate(cfg, sim, step_callback=lambda step, d: steps.__setitem__(step, d.copy()))
    mesh = r["outputs"]["mesh"]
    np.testing.assert_array_equal(mesh["V"], sim["V"])  # at rest
    for step in [0, 2, 4]:
        np.testing.assert_allclose(mesh["vertex_attributes"][f"displacement_{step}"],
                                   steps[step] / 1e-3, rtol=0, atol=1e-7)
    assert r["report"]["max_displacement"] > 0
    assert "displacement_1" not in mesh["vertex_attributes"]

    again = Pipeline([stage("sim", "polyfem_sim", SIMULATION)], str(tmp_path),
                     inputs={"m": boxes3d.name}).run()["sim"]
    assert again["source"] == "cache" and again["report"]["steps"] == [0, 2, 4]
    for step in [0, 2, 4]:
        name = f"displacement_{step}"
        np.testing.assert_array_equal(
            np.asarray(again["outputs"]["mesh"]["vertex_attributes"][name]["values"]),
            mesh["vertex_attributes"][name])
