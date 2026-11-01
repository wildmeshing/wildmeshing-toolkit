"""The home page's example (docs/index.md): a tet mesh of the bunny, in memory."""

import os
from pathlib import Path

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))

# --8<-- [start:quickstart]
import wildmeshing as wm

bunny = wm.read_mesh(str(DATA / "bunny.obj"))
outputs, report = wm.tetwild({"input": ["bunny"], "num_threads": 4}, {"bunny": bunny})
print(report["#t"], "tets, max energy", report["max_energy"])
wm.write_mesh("bunny_tets.vtu", outputs["mesh"])
# --8<-- [end:quickstart]
