"""The gallery's pipelines (docs/gallery/examples.yaml, `scene: <function>`).

Each function runs a documentation example and returns the meshes to show, {"input": mesh,
"output": mesh}, with the run's report. The workflows run their own example script
(docs/examples/python) and take the meshes from its variables, so the gallery shows exactly what
the page does. Called by docs/tools/build_gallery.py, from a scratch working directory.
"""

import os
import runpy
import tempfile
from pathlib import Path

import numpy as np

EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "python"


def run_example(name, data):
    """Run docs/examples/python/<name>.py in a scratch directory; its variables."""
    os.environ["WMTK_DATA"] = str(Path(data).resolve())
    cwd = os.getcwd()
    with tempfile.TemporaryDirectory() as scratch:
        os.chdir(scratch)
        try:
            return runpy.run_path(str(EXAMPLES / f"{name}.py"), run_name="__gallery__")
        finally:
            os.chdir(cwd)


def merge(*meshes):
    """One mesh of several, keeping each one's cell blocks."""
    Vs, blocks, offset = [], {}, 0
    for mesh in meshes:
        Vs.append(np.asarray(mesh["V"], dtype=np.float64))
        for kind, block in mesh["cells"].items():
            blocks.setdefault(kind, []).append(dict(block, connectivity=np.asarray(block["connectivity"]) + offset))
        offset += len(Vs[-1])
    cells = {}
    for kind, parts in blocks.items():
        cells[kind] = {"connectivity": np.vstack([p["connectivity"] for p in parts])}
        if len(parts) == 1 and "attributes" in parts[0]:
            cells[kind]["attributes"] = parts[0]["attributes"]
    return {"V": np.vstack(Vs), "cells": cells}


def transformed(mesh, M):
    M = np.asarray(M, dtype=np.float64)
    V = np.asarray(mesh["V"], dtype=np.float64) @ M[:3, :3].T + M[:3, 3]
    return dict(mesh, V=V)


def checker(P, size):
    """Two colors in a 3D checkerboard of cubes of the given size: which one each point is in."""
    return (np.floor(P / size).astype(np.int64).sum(axis=1) % 2).astype(np.uint8)


def painted(V, F, side):
    """A triangle mesh whose two regions are the two colors of `side` (one per triangle)."""
    values = np.stack([side == 0, side == 1], axis=1).astype(np.uint8)
    return {"V": V, "cells": {"triangle": {"connectivity": np.asarray(F), "attributes": {
        "tags": {"values": values, "labels": ["checker A", "checker B"]}}}}}


def tracking(data):
    """The dense bunny painted with a checkerboard, and the tet mesh's surface painted through the
    two composed maps: each of its triangles takes the color of its centroid's preimage."""
    example = run_example("tracking_correspondences", data)
    r = example["run"](example["parse_args"](["--no-gui"]))
    size = r.diag / 12
    dense_side = checker(r.V_in[r.F_in].mean(axis=1), size)

    # Each tracked triangle's centroid, by its three vertices, back onto the remeshed surface and
    # then onto the dense input.
    third = np.full((len(r.S_t), 3), 1 / 3)
    face, bary = r.remesh_map.to_input(*r.tet_map.to_input(np.asarray(r.S_t), third))
    found = face >= 0
    P = np.zeros((len(face), 3))
    P[found] = np.einsum("nk,nkd->nd", bary[found], r.V_in[r.F_in[face[found]]])
    tet_side = checker(P, size)
    tet_side[~found] = 2  # no preimage: neither color
    return {"input": painted(r.V_in, r.F_in, dense_side),
            "output": painted(r.V_t, r.S_t, tet_side)}, {"#t": int(len(r.T_t))}


def workflow_regions(data):
    """Two spheres meshed into regions, the overlap renamed, a gap offset around one of them, and
    the result remeshed."""
    v = run_example("workflow_regions", data)
    spheres = merge(v["sphere"], transformed(v["sphere"], v["shift"]))
    return {"input": spheres, "output": v["m4"]["mesh"]}, v["report"]


def workflow_repair(data):
    """Two touching spheres made manifold, then remeshed."""
    v = run_example("workflow_repair", data)
    return {"input": v["spheres"], "output": v["remeshed"]["mesh"]}, v["report"]


def workflow_tetwild_features(data):
    """The feature curves of a cube, and the tet mesh of the cube that keeps them: its feature
    edges drawn over it."""
    v = run_example("workflow_tetwild_features", data)
    inputs = v["inputs"]
    # The curves alone: inside the cube, its surface would hide them.
    shown_input = inputs["curves"]
    tets = {"V": v["mesh"]["V"], "cells": {"tetrahedron": v["mesh"]["cells"]["tetrahedron"]}}
    shown_output = merge(tets, {"V": v["features"]["V"], "cells": {"edge": v["features"]["cells"]["edge"]}})
    return {"input": shown_input, "output": shown_output}, v["report"]
