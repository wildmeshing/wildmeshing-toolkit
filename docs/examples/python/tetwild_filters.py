"""The TetWild tutorial (docs/tutorials/tetwild.md): what each filter keeps of a closed sphere, of
the same sphere with a hole, and of a surface whose triangles are not consistently oriented."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


def volume(mesh):
    """The total volume of the tets of a mesh dictionary."""
    V, T = mesh["V"], mesh["cells"]["tetrahedron"]["connectivity"]
    a, b, c, d = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]], V[T[:, 3]]
    return np.abs(np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a)).sum() / 6


def keep(surface, filter):
    """What one filter keeps of a surface: (tets, volume), or the error."""
    # Coarse (length_rel 0.1) and sequential, so that the runs are short and repeat exactly.
    options = {"input": ["s"], "filter": filter, "length_rel": 0.1, "num_threads": 0}
    try:
        outputs, _ = wm.tetwild(options, {"s": surface})
    except RuntimeError as error:
        return str(error)
    mesh = outputs["mesh"]
    return f"{len(mesh['cells']['tetrahedron']['connectivity'])} tets, volume {volume(mesh):.3f}"


# --8<-- [start:open]
# A sphere of radius 0.7, closed, and the same sphere without its top cap.
sphere = wm.read_mesh(str(DATA / "sphere.obj"))
V, F = sphere["V"], sphere["cells"]["triangle"]["connectivity"]
cap = V[F].mean(axis=1)[:, 2] > 0.49
opened = {"V": V, "cells": {"triangle": {"connectivity": F[~cap]}}}
print(f"removed {cap.sum()} of {len(F)} triangles")
for name, surface in [("closed", sphere), ("open", opened)]:
    for filter in ["none", "flood", "input", "tracked"]:
        print(f"{name} sphere, filter {filter}: {keep(surface, filter)}")
# --8<-- [end:open]

# --8<-- [start:orientation]
# Two intersecting spheres whose triangles are oriented at random.
spheres = wm.read_mesh(str(DATA / "double_sphere.obj"))
for filter in ["flood", "input", "tracked"]:
    print(f"double sphere, filter {filter}: {keep(spheres, filter)}")
# --8<-- [end:orientation]
