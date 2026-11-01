"""Getting started, calling components (docs/getting-started/calling-components.md): mesh
dictionaries, the two routes, reports, and files."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))

# --8<-- [start:dict]
# Two triangles of the unit square, as a mesh dictionary: vertex positions (3 columns, 2D meshes
# have z = 0) and cell blocks by type.
square = {
    "V": np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=np.float64),
    "cells": {
        "triangle": {"connectivity": np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64)},
    },
}
# --8<-- [end:dict]
wm.write_mesh("square.vtu", square)

# --8<-- [start:routes]
sphere = wm.read_mesh(str(DATA / "sphere.obj"))

# Memory: inputs by name, nothing written; returns the output meshes and a report.
outputs, report = wm.qslim({"input": "sphere", "target_rel": 0.5}, {"sphere": sphere})
print(list(outputs), report["vertices"], "vertices")

# Files: as wmtk_app does with the same options; returns nothing.
wm.qslim({"input": str(DATA / "sphere.obj"), "output": "sphere_half.obj", "target_rel": 0.5})
# --8<-- [end:routes]

# --8<-- [start:files]
mesh = outputs["mesh"]
wm.write_mesh("sphere_half.msh", mesh)   # .msh or .vtu
again = wm.read_mesh("sphere_half.msh")  # .msh, .obj, .stl, .off, ...
assert np.allclose(again["V"], mesh["V"])

# A checkpoint: every output mesh of a result, read back exactly as written.
wm.save(outputs, "checkpoint/sphere_half")
restored = wm.load("checkpoint/sphere_half")
# --8<-- [end:files]

# --8<-- [start:history]
outputs, report = wm.qslim(
    {"input": "sphere", "target_rel": 0.5, "history_output": "record"}, {"sphere": sphere})
history = report["history"]  # bytes: the record of every collapse
tracking = wm.Tracking(history)
print(tracking.build_warnings)
# --8<-- [end:history]
