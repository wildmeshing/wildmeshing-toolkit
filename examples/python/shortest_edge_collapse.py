"""The shortest edge collapse tutorial (docs/tutorials/shortest_edge_collapse.md): simplify the
Octocat to a tenth of its vertices inside an envelope, on files and in memory; then see what the
envelope, its sampled test, the link condition and the boundary tube change."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


def open_edges(mesh):
    """The boundary edges (one triangle) and the non-manifold ones (three or more) of a mesh."""
    F = mesh["cells"]["triangle"]["connectivity"]
    E = np.sort(F[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    _, count = np.unique(E, axis=0, return_counts=True)
    return int((count == 1).sum()), int((count > 2).sum())


# --8<-- [start:file]
wm.shortest_edge_collapse({
    "input": str(DATA / "Octocat.obj"),
    "output": "octocat_sec.obj",
    "target_rel": 0.1,
    "eps_rel": 0.002,
    "num_threads": 4,
})
# --8<-- [end:file]

# --8<-- [start:memory]
octocat = wm.read_mesh(str(DATA / "Octocat.obj"))
outputs, report = wm.shortest_edge_collapse(
    {"input": "octocat", "target_rel": 0.1, "eps_rel": 0.002, "num_threads": 4},
    {"octocat": octocat},
)
simplified = outputs["mesh"]
print(len(octocat["V"]), "->", len(simplified["V"]), "vertices")
# --8<-- [end:memory]

# --8<-- [start:sweep]
# The envelope against the target: with a thin envelope the collapses stop before the target.
for eps_rel in [-1, 0.01, 0.002]:
    outputs, _ = wm.shortest_edge_collapse(
        {"input": "octocat", "target_rel": 0.01, "eps_rel": eps_rel, "num_threads": 4},
        {"octocat": octocat})
    print(f"eps_rel {eps_rel:g}: {len(outputs['mesh']['V'])} vertices")
# --8<-- [end:sweep]

# --8<-- [start:sampled]
# The same envelopes, tested by sampling.
for eps_rel in [0.01, 0.002]:
    outputs, _ = wm.shortest_edge_collapse(
        {"input": "octocat", "target_rel": 0.01, "eps_rel": eps_rel, "num_threads": 4,
         "use_sample_envelope": True},
        {"octocat": octocat})
    print(f"sampled, eps_rel {eps_rel:g}: {len(outputs['mesh']['V'])} vertices")
# --8<-- [end:sampled]

# --8<-- [start:link]
# Without the link condition a collapse may change the topology: the closed Octocat opens up.
outputs, _ = wm.shortest_edge_collapse(
    {"input": "octocat", "target_rel": 0.01, "eps_rel": 0.01, "num_threads": 4,
     "use_link_condition": False},
    {"octocat": octocat})
boundary, non_manifold = open_edges(outputs["mesh"])
print(f"{len(outputs['mesh']['V'])} vertices, {boundary} boundary edges, "
      f"{non_manifold} non-manifold edges")
# --8<-- [end:link]

# --8<-- [start:boundary]
# The bunny has holes. Frozen (the default), its boundary keeps every vertex; in a tube, it is
# simplified too.
bunny = wm.read_mesh(str(DATA / "bunny.obj"))
print("input:", open_edges(bunny)[0], "boundary edges")
for boundary_eps_rel in [-1, 0.002]:
    outputs, _ = wm.shortest_edge_collapse(
        {"input": "bunny", "target_rel": 0.1, "boundary_eps_rel": boundary_eps_rel},
        {"bunny": bunny})
    print(f"boundary_eps_rel {boundary_eps_rel:g}: {len(outputs['mesh']['V'])} vertices, "
          f"{open_edges(outputs['mesh'])[0]} boundary edges")
# --8<-- [end:boundary]
