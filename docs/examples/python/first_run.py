"""Getting started, first run (docs/getting-started/first-run.md): isotropic remeshing of the
bunny from Python. The same run as json/first_run.json and cpp/first_run.cpp."""

import os
from pathlib import Path

import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))

# --8<-- [start:file]
# On files: the same options as the JSON file, with paths relative to the working directory.
wm.isotropic_remeshing({
    "input": str(DATA / "bunny.obj"),
    "output": "bunny_remeshed.obj",
    "length_rel": 0.02,
})
# --8<-- [end:file]

# --8<-- [start:memory]
# In memory: inputs by name, NumPy arrays in and out, nothing written.
outputs, report = wm.isotropic_remeshing(
    {"input": "bunny", "length_rel": 0.02},
    {"bunny": wm.read_mesh(str(DATA / "bunny.obj"))},
)
mesh = outputs["mesh"]
print(len(mesh["V"]), "vertices,", len(mesh["cells"]["triangle"]["connectivity"]), "faces")
wm.write_mesh("bunny_remeshed.msh", mesh)
# --8<-- [end:memory]
