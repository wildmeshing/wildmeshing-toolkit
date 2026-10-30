"""The manifold extraction tutorial (docs/tutorials/manifold_extraction.md): two regions that touch
along a circle, made into a selection whose boundary is manifold; in 3D and in 2D."""

import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))

# --8<-- [start:file]
wm.manifold_extraction({
    "input": str(DATA / "double_sphere_hole.msh"),
    "output": "double_sphere_manifold",
    "tag_selection": "tag_0 | tag_1",
    "fill_tags": [],
    "write_surface": True,
})
# --8<-- [end:file]

# --8<-- [start:memory]
spheres = wm.read_mesh(str(DATA / "double_sphere_hole.msh"))
outputs, report = wm.manifold_extraction(
    {"input": "spheres", "tag_selection": "tag_0 | tag_1", "fill_tags": []},
    {"spheres": spheres},
)
repaired = outputs["mesh"]
print(report["before #t"], "->", report["after #t"], "tets")
# --8<-- [end:memory]


# --8<-- [start:check]
def selected(mesh, *tags):
    """The tets that carry any of `tags`."""
    t = mesh["cells"]["tetrahedron"]["attributes"]["tags"]
    columns = [t["labels"].index(tag) for tag in tags]
    return np.asarray(t["values"])[:, columns].any(axis=1)


def boundary(mesh, inside):
    """The triangles between the tets in `inside` and the others (or the outside)."""
    T = np.asarray(mesh["cells"]["tetrahedron"]["connectivity"])[inside]
    F = np.sort(np.concatenate([T[:, [0, 1, 2]], T[:, [0, 1, 3]], T[:, [0, 2, 3]],
                                T[:, [1, 2, 3]]]), axis=1)
    faces, count = np.unique(F, axis=0, return_counts=True)
    return faces[count == 1]


def nonmanifold_edges(F):
    """Edges of the triangles F that are not on exactly two of them."""
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    _, count = np.unique(E, axis=0, return_counts=True)
    return int((count != 2).sum())


before = boundary(spheres, selected(spheres, "tag_0", "tag_1"))
after = boundary(repaired, selected(repaired, "tag_0", "tag_1"))
print("non-manifold edges:", nonmanifold_edges(before), "->", nonmanifold_edges(after))
print("extracted surface:", len(outputs["surface"]["cells"]["triangle"]["connectivity"]),
      "triangles, non-manifold edges:",
      nonmanifold_edges(outputs["surface"]["cells"]["triangle"]["connectivity"]))
# --8<-- [end:check]

# --8<-- [start:fill]
# Fill the carved tube with tag_0 instead of ambient: the two spheres are joined, not separated.
for fill in [[], ["tag_0"]]:
    out, _ = wm.manifold_extraction(
        {"input": "spheres", "tag_selection": "tag_0 | tag_1", "fill_tags": fill},
        {"spheres": spheres})
    m = out["mesh"]
    print(f"fill_tags {fill}: tag_0 {selected(m, 'tag_0').sum()} tets,"
          f" tag_1 {selected(m, 'tag_1').sum()} tets,"
          f" non-manifold edges {nonmanifold_edges(boundary(m, selected(m, 'tag_0', 'tag_1')))}")
# --8<-- [end:fill]

# --8<-- [start:planar]
# Two triangles of tag_0 that share a vertex and nothing else.
triangles = wm.read_mesh(str(DATA / "vertex_adj_tris.msh"))
out, report = wm.manifold_extraction(
    {"input": "triangles", "tag_selection": "tag_0"}, {"triangles": triangles})
curve = out["surface"]  # the boundary of tag_0 after the repair: edges
print(report["before #f"], "->", report["after #f"], "triangles;",
      len(curve["cells"]["edge"]["connectivity"]), "boundary edges")
# --8<-- [end:planar]
