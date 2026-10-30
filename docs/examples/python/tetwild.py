"""The TetWild tutorial (docs/tutorials/tetwild.md): tetrahedralize the inside of a Thingi10K
snowman, on files and in memory; then the options the page measures."""

import os
from pathlib import Path

import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))

# --8<-- [start:file]
wm.tetwild({
    "input": [str(DATA / "thingi_366725.stl")],
    "output": "snowman",
    "filter": "tracked",
    "stop_energy": 10,
    "num_threads": 4,
    "report": "snowman_report.json",
})
# --8<-- [end:file]

# --8<-- [start:memory]
snowman = wm.read_mesh(str(DATA / "thingi_366725.stl"))
outputs, report = wm.tetwild(
    {"input": ["snowman"], "filter": "tracked", "stop_energy": 10, "num_threads": 4},
    {"snowman": snowman},
)
mesh = outputs["mesh"]
tets = mesh["cells"]["tetrahedron"]["connectivity"]
surface = outputs["surface"]["cells"]["triangle"]["connectivity"]
print(len(mesh["V"]), "vertices,", len(tets), "tets,", len(surface), "surface triangles,",
      f"max energy {report['max_energy']:.1f}, avg {report['avg_energy']:.2f}")
# --8<-- [end:memory]

# --8<-- [start:sweep]
# One option at a time against the first run. With 4 threads the numbers vary a little from run
# to run; num_threads 0 makes them repeat exactly, in about twice the time.
for change in [{}, {"stop_energy": 100}, {"coarsen_pass": True}, {"length_rel": 0.1},
               {"eps_rel": 0.005}]:
    options = {"input": ["snowman"], "filter": "tracked", "stop_energy": 10, "num_threads": 4,
               "DEBUG_hausdorff": True, **change}
    outputs, report = wm.tetwild(options, {"snowman": snowman})
    print(f"{change}: {report['#v']} vertices, {report['#t']} tets, "
          f"max energy {report['max_energy']:.1f}, avg {report['avg_energy']:.2f}, "
          f"iterations {report['#iterations']}, "
          f"surface within {report['hausdorff']:.3f} of the input (eps {report['eps']:.3f})")
# --8<-- [end:sweep]
