"""The repair-before-meshing workflow (docs/tutorials/workflows/repair-before-meshing.md): make
the boundary of two touching spheres manifold, then remesh for quality, in memory."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


def selected(mesh, *tags):
    """The tets that carry any of `tags`."""
    t = mesh["cells"]["tetrahedron"]["attributes"]["tags"]
    columns = [t["labels"].index(tag) for tag in tags]
    return np.asarray(t["values"])[:, columns].any(axis=1)


def nonmanifold_edges(mesh, *tags):
    """Edges of the boundary of the union of `tags` that are not on exactly two of its faces."""
    T = np.asarray(mesh["cells"]["tetrahedron"]["connectivity"])[selected(mesh, *tags)]
    F = np.sort(np.concatenate([T[:, [0, 1, 2]], T[:, [0, 1, 3]], T[:, [0, 2, 3]],
                                T[:, [1, 2, 3]]]), axis=1)
    faces, count = np.unique(F, axis=0, return_counts=True)
    F = faces[count == 1]
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    _, count = np.unique(E, axis=0, return_counts=True)
    return int((count != 2).sum())


# --8<-- [start:chain]
spheres = wm.read_mesh(str(DATA / "double_sphere_hole.msh"))
print("input: non-manifold edges", nonmanifold_edges(spheres, "tag_0", "tag_1"))

# 1. Carve a thin tube of background around the circle where the spheres touch.
repaired, _ = wm.manifold_extraction(
    {"input": "spheres", "tag_selection": "tag_0 | tag_1", "fill_tags": []},
    {"spheres": spheres})
print("repaired: non-manifold edges", nonmanifold_edges(repaired["mesh"], "tag_0", "tag_1"))

# 2. Remesh for quality; the regions keep their topology, so the boundary stays manifold.
remeshed, report = wm.simwild(
    {"input": ["repaired"], "operation": "remeshing", "num_threads": 4},
    {"repaired": repaired["mesh"]})
mesh = remeshed["mesh"]
print("remeshed: non-manifold edges", nonmanifold_edges(mesh, "tag_0", "tag_1"),
      f"| {report['#t']} tets, max energy {report['max_energy']:.3g}")
# --8<-- [end:chain]
