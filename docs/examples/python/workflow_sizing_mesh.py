"""The sizing mesh workflow (docs/tutorials/workflows/sizing-mesh.md): a largest edge length that
varies over space, for TriWild (a triangle mesh in the plane), TetWild (a tet mesh) and isotropic
remeshing (a surface)."""

import itertools
import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


def against(mesh, block, field):
    """Each edge of a cell block against the field at its midpoint: the share of the edges longer
    than 4/3 of it (those a split would cut), and the median ratio."""
    V, C = mesh["V"], mesh["cells"][block]["connectivity"]
    E = np.concatenate([C[:, [i, j]] for i, j in itertools.combinations(range(C.shape[1]), 2)])
    E = np.unique(np.sort(E, axis=1), axis=0)
    ratio = np.linalg.norm(V[E[:, 0]] - V[E[:, 1]], axis=1) / field(0.5 * (V[E[:, 0]] + V[E[:, 1]]))
    return f"{100 * np.mean(ratio > 4 / 3):.1f}% of the edges past 4/3 of the field, " \
           f"median {np.median(ratio):.2f} of it"


# --8<-- [start:build]
# A square around the input, two triangles, whose length grows from `left` on its left side to
# `right` on its right: interpolated linearly in between.
def planar_sizing(lo, hi, left, right):
    V = np.array([[lo, lo, 0], [hi, lo, 0], [hi, hi, 0], [lo, hi, 0]], dtype=float)
    F = np.array([[0, 1, 2], [0, 2, 3]])
    return {
        "V": V,
        "cells": {"triangle": {"connectivity": F}},
        "vertex_attributes": {"target_length": np.array([left, right, right, left])},
    }


# The lengths are in the input's units: from 1% of the circle's size to 6%. The square reaches a
# fifth of that size past the circle on each side, beyond the background mesh TriWild builds
# around its input: a point outside every triangle takes the global length.
circle = wm.read_mesh(str(DATA / "triwild_circle.obj"))
lo, hi = circle["V"][:, :2].min(), circle["V"][:, :2].max()
size = hi - lo
lo, hi = lo - 0.2 * size, hi + 0.2 * size
sizing = planar_sizing(lo, hi, 0.01 * size, 0.06 * size)
# --8<-- [end:build]


def field(points):
    """The sizing mesh's length at points: linear in x."""
    return np.interp(points[:, 0], [lo, hi], [0.01 * size, 0.06 * size])


# --8<-- [start:triwild]
# The inside of the circle, with and without the sizing mesh, sequentially (num_threads 0) so
# that the numbers repeat exactly.
options = {"input": ["circle"], "filter": "input", "stop_energy": 10, "num_threads": 0}
for name, extra in [("length_rel 0.05", {}), ("sizing mesh", {"sizing_mesh": "sizing"})]:
    outputs, report = wm.triwild({**options, **extra}, {"circle": circle, "sizing": sizing})
    triangles = len(outputs["mesh"]["cells"]["triangle"]["connectivity"])
    print(f"triwild, {name}: {triangles} triangles after {report['#iterations']} iteration(s), "
          f"{against(outputs['mesh'], 'triangle', field)}")
# --8<-- [end:triwild]

# --8<-- [start:file]
# The same run on files: the sizing mesh is a mesh file whose vertices carry the field
# target_length, relative to input_dir like the inputs.
wm.write_mesh("sizing.msh", sizing)
wm.triwild({**options, "input": [str(DATA / "triwild_circle.obj")], "sizing_mesh": "sizing.msh",
            "output": "circle"})
print(f"triwild on files: {against(wm.read_mesh('circle.msh'), 'triangle', field)}")
# --8<-- [end:file]

# --8<-- [start:limit]
# With a sizing mesh, limit_collapse_length is on unless set: no collapse leaves an edge longer
# than 4/3 of its target. Turned off, collapses merge edges past the field:
for limit in [True, False]:
    outputs, report = wm.triwild(
        {**options, "sizing_mesh": "sizing", "limit_collapse_length": limit},
        {"circle": circle, "sizing": sizing})
    triangles = len(outputs["mesh"]["cells"]["triangle"]["connectivity"])
    print(f"limit_collapse_length {limit}: {triangles} triangles after {report['#iterations']} "
          f"iteration(s), {against(outputs['mesh'], 'triangle', field)}")
# --8<-- [end:limit]

# --8<-- [start:tetwild]
# TetWild: the sizing mesh is a tet mesh. A box around a sphere of radius 0.7, as five tets,
# whose length grows from 0.03 at its bottom to 0.15 at its top: linear in z.
def box_sizing(lo, hi, bottom, top):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    V = np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)], dtype=float)
    T = np.array([[0, 1, 3, 5], [0, 3, 2, 6], [0, 5, 4, 6], [3, 5, 6, 7], [0, 3, 6, 5]])
    return {"V": V, "cells": {"tetrahedron": {"connectivity": T}},
            "vertex_attributes": {"target_length": np.where(V[:, 2] == z0, bottom, top)}}


sphere = wm.read_mesh(str(DATA / "sphere.obj"))
margin = 0.2 * np.linalg.norm(sphere["V"].max(axis=0) - sphere["V"].min(axis=0))
box_lo, box_hi = sphere["V"].min(axis=0) - margin, sphere["V"].max(axis=0) + margin
box = box_sizing(box_lo, box_hi, 0.03, 0.15)
for name, extra in [("length_rel 0.05", {}), ("sizing mesh", {"sizing_mesh": "box"})]:
    outputs, report = wm.tetwild(
        {"input": ["sphere"], "filter": "tracked", "num_threads": 0, **extra},
        {"sphere": sphere, "box": box})
    tets = len(outputs["mesh"]["cells"]["tetrahedron"]["connectivity"])
    print(f"tetwild, {name}: {tets} tets, " + against(
        outputs["mesh"], "tetrahedron", lambda p: np.interp(p[:, 2], [box_lo[2], box_hi[2]],
                                                            [0.03, 0.15])))
# --8<-- [end:tetwild]

# --8<-- [start:isotropic]
# Isotropic remeshing: the sizing mesh is a surface, read at the point closest to each vertex.
# Here it is the input itself, with a field that goes from 4% of the bunny's diagonal at its
# feet to 1% at its ears: linear in y.
bunny = wm.read_mesh(str(DATA / "bunny.obj"))
y0, y1 = bunny["V"][:, 1].min(), bunny["V"][:, 1].max()
diag = np.linalg.norm(bunny["V"].max(axis=0) - bunny["V"].min(axis=0))
height = (bunny["V"][:, 1] - y0) / (y1 - y0)
bunny_sizing = dict(bunny, vertex_attributes={"target_length": diag * (0.04 - 0.03 * height)})
for name, extra in [("length_rel 0.025", {"length_rel": 0.025}),
                    ("sizing mesh", {"sizing_mesh": "field"})]:
    outputs, _ = wm.isotropic_remeshing({"input": "bunny", "max_iterations": 5, **extra},
                                        {"bunny": bunny, "field": bunny_sizing})
    triangles = len(outputs["mesh"]["cells"]["triangle"]["connectivity"])
    print(f"isotropic_remeshing, {name}: {triangles} triangles, " + against(
        outputs["mesh"], "triangle", lambda p: diag * np.interp(p[:, 1], [y0, y1], [0.04, 0.01])))
# --8<-- [end:isotropic]
