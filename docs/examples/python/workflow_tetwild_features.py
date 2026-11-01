"""The workflow TetWild with features (docs/tutorials/workflows/tetwild-features.md): a cube with a
feature curve and feature points inside, forced into the tet mesh; then check what was kept."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))

# --8<-- [start:run]
inputs = {
    "cube": wm.read_mesh(str(DATA / "tetwild_features_cube.obj")),
    "curves": wm.read_mesh(str(DATA / "tetwild_features_edges.obj")),
    "points": wm.read_mesh(str(DATA / "tetwild_features_points.obj")),
}
options = {"input": ["cube"], "input_edges": ["curves"], "input_points": ["points"],
           "filter": "tracked", "stop_energy": 10, "num_threads": 0}
outputs, report = wm.tetwild(options, inputs)
mesh, features = outputs["mesh"], outputs["features"]
print(len(mesh["V"]), "vertices,", len(mesh["cells"]["tetrahedron"]["connectivity"]), "tets,",
      f"max energy {report['max_energy']:.2f}")
print(len(features["cells"]["edge"]["connectivity"]), "tet edges on the curve,",
      len(features["cells"]["point"]["connectivity"]), "anchored points")
# --8<-- [end:run]

# --8<-- [start:check]
curve_V = inputs["curves"]["V"]
curve_E = inputs["curves"]["cells"]["edge"]["connectivity"]
segments = [(curve_V[a], curve_V[b]) for a, b in curve_E]
# The points to keep: the curve's ends (one edge), the vertex of the curve file that no edge uses,
# and the points of the point file.
valence = np.bincount(curve_E.ravel(), minlength=len(curve_V))
anchors = np.vstack([curve_V[valence < 2], inputs["points"]["V"]])


def distance_to(p, segs):
    """The distance from a point to the nearest of some segments."""
    best = np.inf
    for a, b in segs:
        t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-300), 0, 1)
        best = min(best, np.linalg.norm(p - (a + t * (b - a))))
    return best


def check(mesh, features):
    V = mesh["V"]
    # Each anchor: how far is the nearest vertex of the tet mesh?
    worst_anchor = max(np.linalg.norm(V - p, axis=1).min() for p in anchors)
    # The tet edges tagged as the curve: how far do they stray from the input curve
    # (containment), and how far is the input curve from them (coverage)?
    FV = features["V"]
    tagged = [(FV[a], FV[b]) for a, b in features["cells"]["edge"]["connectivity"]]
    samples = np.linspace(0, 1, 17)
    containment = max(distance_to(a + s * (b - a), segments) for a, b in tagged for s in samples)
    coverage = max(distance_to(a + s * (b - a), tagged) for a, b in segments for s in samples)
    return worst_anchor, containment, coverage


tube = 0.5 * report["eps"]  # feature_envelope_ratio (0.5) times eps
worst_anchor, containment, coverage = check(mesh, features)
print(f"anchors: the worst is {worst_anchor:.2g} from a vertex (tube radius {tube:.2g})")
print(f"curve: tagged edges within {containment:.2g} of it, covering it to {coverage:.2g}")
# --8<-- [end:check]

# --8<-- [start:erosion]
# Without the anchors, the curve's free end and the points may move away.
outputs, report = wm.tetwild({**options, "preserve_feature_points": False}, inputs)
worst_anchor, containment, coverage = check(outputs["mesh"], outputs["features"])
print(f"preserve_feature_points false: the worst anchor is {worst_anchor:.2g} from a vertex, "
      f"the curve is covered to {coverage:.2g}")
# --8<-- [end:erosion]
