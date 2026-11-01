"""The QSlim tutorial (docs/tutorials/qslim.md): simplify the Octocat to a tenth of its vertices,
inside an envelope, on files and in memory, and see what the envelope costs."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))

# --8<-- [start:file]
wm.qslim({
    "input": str(DATA / "Octocat.obj"),
    "output": "octocat_qslim.obj",
    "target_rel": 0.1,
    "eps_rel": 0.002,
    "num_threads": 4,
})
# --8<-- [end:file]

# --8<-- [start:memory]
octocat = wm.read_mesh(str(DATA / "Octocat.obj"))
outputs, report = wm.qslim(
    {"input": "octocat", "target_rel": 0.1, "eps_rel": 0.002, "num_threads": 4},
    {"octocat": octocat},
)
simplified = outputs["mesh"]
print(len(octocat["V"]), "->", len(simplified["V"]), "vertices")
# --8<-- [end:memory]

# --8<-- [start:sweep]
# The envelope against the target: with a thin envelope the collapses stop before the target.
for eps_rel in [-1, 0.01, 0.002]:
    outputs, _ = wm.qslim(
        {"input": "octocat", "target_rel": 0.01, "eps_rel": eps_rel, "num_threads": 4},
        {"octocat": octocat})
    print(f"eps_rel {eps_rel:g}: {len(outputs['mesh']['V'])} vertices")
# --8<-- [end:sweep]
