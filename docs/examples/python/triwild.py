"""The TriWild tutorial (docs/tutorials/triwild.md): triangulate five overlapping closed curves,
each an input with a name, on files and in memory; then the options the page measures."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))
NAMES = ["blob1", "blob2", "circle", "ellipse", "rectangle"]

# --8<-- [start:file]
wm.triwild({
    "input": [str(DATA / f"triwild_{name}.obj") for name in NAMES],
    "input_names": NAMES,
    "output": "shapes",
    "filter": "input",
    "stop_energy": 10,
    "num_threads": 4,
    "report": "shapes_report.json",
})
# --8<-- [end:file]

# --8<-- [start:memory]
curves = {name: wm.read_mesh(str(DATA / f"triwild_{name}.obj")) for name in NAMES}
outputs, report = wm.triwild(
    {"input": NAMES, "input_names": NAMES, "filter": "input", "stop_energy": 10,
     "num_threads": 4},
    curves,
)
mesh = outputs["mesh"]
triangles = mesh["cells"]["triangle"]
print(len(mesh["V"]), "vertices,", len(triangles["connectivity"]), "triangles,",
      f"max energy {report['max_energy']:.2f}")
# One column per input: 1 where the triangle lies inside that curve.
tags = triangles["attributes"]["tags"]
for label, count in zip(tags["labels"], np.asarray(tags["values"]).sum(axis=0)):
    print(f"  {label}: {count} triangles")
# --8<-- [end:memory]


def triangulate(options, meshes=curves, inputs=NAMES):
    """One run in memory, sequential so that the numbers repeat exactly."""
    options = {"input": inputs, "num_threads": 0, **options}
    return wm.triwild(options, meshes)


# --8<-- [start:sweep]
# One option at a time against the first run (filter input, stop_energy 10).
base = {"filter": "input", "stop_energy": 10}
for change in [{}, {"stop_energy": 100}, {"stop_energy": 5}, {"stop_energy": 3},
               {"eps_rel": 0.0002}, {"eps_rel": 0.005},
               {"length_rel": 0.02}, {"length_rel": 0.01},
               {"coarsen_pass": True}]:
    outputs, report = triangulate({**base, **change, "DEBUG_hausdorff": True})
    print(f"{change}: {report['#t']} triangles, max energy {report['max_energy']:.2f}, "
          f"avg {report['avg_energy']:.2f}, iterations {report['#iterations']}, "
          f"curves within {report['hausdorff']:.3f} of the input (eps {report['eps']:.3f})")
# --8<-- [end:sweep]

# --8<-- [start:points]
# A closed curve and twelve points to keep as vertices: nine inside, three outside.
star = {"curve": wm.read_mesh(str(DATA / "triwild_free_points_curve.obj")),
        "points": wm.read_mesh(str(DATA / "triwild_free_points_points.obj"))}
outputs, report = triangulate(
    {"input_points": ["points"], "stop_energy": 10, "DEBUG_feature_retention": True},
    star, ["curve"])
V = outputs["mesh"]["V"]
# How far each input point is from the nearest vertex of the output.
worst = max(np.linalg.norm(V - p, axis=1).min() for p in star["points"]["V"])
print(f"{report['features_retained']} of {report['features_total']} points kept, the worst "
      f"{worst:.3f} from a vertex (eps {report['eps']:.3f})")
# --8<-- [end:points]

# --8<-- [start:open]
# Open curves: 46 polylines with 94 loose ends. The filters do not agree on what is inside.
drawing = {"drawing": wm.read_mesh(str(DATA / "triwild_polylines.obj"))}
for filter in ["none", "flood", "input", "tracked"]:
    outputs, report = triangulate({"filter": filter}, drawing, ["drawing"])
    V, F = outputs["mesh"]["V"], outputs["mesh"]["cells"]["triangle"]["connectivity"]
    e1, e2 = V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]
    area = 0.5 * np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]).sum()
    print(f"filter {filter}: {len(F)} triangles, area {area:.0f}")
# --8<-- [end:open]
