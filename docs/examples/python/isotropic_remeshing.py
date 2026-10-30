"""The isotropic remeshing tutorial (docs/tutorials/isotropic_remeshing.md): remesh the bunny to a
uniform edge length, on files and in memory, and see what the length, the iterations, the
boundary and the swaps change; then keep the sharp edges of a box, as features and as labels,
and remesh a label map."""

import json
import os
from pathlib import Path

import numpy as np
import wildmeshing as wm

# The example meshes: $WMTK_DATA, or data/models from the repository root.
DATA = Path(os.environ.get("WMTK_DATA", "data/models"))


def quality(mesh):
    """The smallest angle of the mesh, in degrees, and the percentage of its triangles with an
    angle under 30 degrees."""
    V, F = mesh["V"], mesh["cells"]["triangle"]["connectivity"]
    angles = []
    for i in range(3):
        u = V[F[:, (i + 1) % 3]] - V[F[:, i]]
        w = V[F[:, (i + 2) % 3]] - V[F[:, i]]
        angles.append(np.degrees(np.arctan2(np.linalg.norm(np.cross(u, w), axis=1),
                                            (u * w).sum(1))))
    smallest = np.min(angles, axis=0)
    return smallest.min(), 100 * (smallest < 30).mean()


def boundary_edges(mesh):
    """The number of edges with one triangle."""
    F = mesh["cells"]["triangle"]["connectivity"]
    E = np.sort(F[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    _, count = np.unique(E, axis=0, return_counts=True)
    return int((count == 1).sum())


# --8<-- [start:file]
wm.isotropic_remeshing({
    "input": str(DATA / "bunny.obj"),
    "output": "bunny_remeshed.obj",
    "length_rel": 0.01,
    "report": "bunny_report.json",
})
# --8<-- [end:file]

# --8<-- [start:memory]
bunny = wm.read_mesh(str(DATA / "bunny.obj"))
outputs, report = wm.isotropic_remeshing({"input": "bunny", "length_rel": 0.01}, {"bunny": bunny})
remeshed = outputs["mesh"]
print(len(bunny["V"]), "->", len(remeshed["V"]), "vertices")
print(f"target length {report['target_length']:.5f}, average {report['avg_length']:.5f}")
# --8<-- [end:memory]

# --8<-- [start:length]
for length_rel in [0.005, 0.01, 0.02]:
    outputs, report = wm.isotropic_remeshing(
        {"input": "bunny", "length_rel": length_rel}, {"bunny": bunny})
    print(f"length_rel {length_rel:g}: {report['#v']} vertices, average edge "
          f"{report['avg_length'] / report['target_length']:.2f} x the target")
# --8<-- [end:length]

# --8<-- [start:iterations]
print("input: smallest angle %.1f, %.2f%% of the triangles under 30 degrees" % quality(bunny))
for max_iterations in [1, 3, 10]:
    outputs, _ = wm.isotropic_remeshing(
        {"input": "bunny", "length_rel": 0.01, "max_iterations": max_iterations},
        {"bunny": bunny})
    print(f"max_iterations {max_iterations}: smallest angle %.2f, %.2f%% under 30 degrees"
          % quality(outputs["mesh"]))
# --8<-- [end:iterations]

# --8<-- [start:boundary]
print("input:", boundary_edges(bunny), "boundary edges")
for options in [{}, {"freeze_boundary": False}, {"boundary_eps_rel": 0.001}]:
    outputs, _ = wm.isotropic_remeshing(
        {"input": "bunny", "length_rel": 0.01, **options}, {"bunny": bunny})
    print(options, boundary_edges(outputs["mesh"]), "boundary edges, smallest angle %.2f" %
          quality(outputs["mesh"])[0])
# --8<-- [end:boundary]

# --8<-- [start:swap]
for options in [{}, {"swap_quality": True}, {"swap_quality": True, "boundary_eps_rel": 0.001}]:
    outputs, _ = wm.isotropic_remeshing(
        {"input": "bunny", "length_rel": 0.01, **options}, {"bunny": bunny})
    print(options, "smallest angle %.2f, %.2f%% under 30 degrees" % quality(outputs["mesh"]))
# --8<-- [end:swap]

# The box: 8 vertices, 12 triangles, and every vertex should stay on its surface.
box = wm.read_mesh(str(DATA / "piece_0.obj"))
lo, hi = box["V"].min(0), box["V"].max(0)


def farthest_from_box(mesh):
    """The largest distance of a vertex of `mesh` from the surface of the box [lo, hi]."""
    P = mesh["V"]
    inside = np.minimum(P - lo, hi - P).min(1)
    outside = np.linalg.norm(np.maximum(np.maximum(lo - P, P - hi), 0), axis=1)
    return np.where(inside > 0, inside, outside).max()


# --8<-- [start:envelope]
for eps_rel in [-1, 0.01, 0.001]:
    outputs, _ = wm.isotropic_remeshing(
        {"input": "box", "length_rel": 0.05, "eps_rel": eps_rel}, {"box": box})
    print(f"eps_rel {eps_rel:g}: farthest vertex {farthest_from_box(outputs['mesh']):.2g} "
          "from the box")
# --8<-- [end:envelope]


# --8<-- [start:sharp]
def sharp_edges(mesh, degrees=30):
    """The edges of a closed manifold mesh whose two triangles' normals are more than `degrees`
    apart."""
    V, F = mesh["V"], mesh["cells"]["triangle"]["connectivity"]
    N = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    N /= np.linalg.norm(N, axis=1, keepdims=True)
    E = np.sort(F[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    face = np.repeat(np.arange(len(F)), 3)
    order = np.lexsort((E[:, 1], E[:, 0]))
    E, face = E[order], face[order]
    # Each edge is there twice, in consecutive rows: once for each of its triangles.
    edges, f0, f1 = E[0::2], face[0::2], face[1::2]
    return edges[(N[f0] * N[f1]).sum(1) < np.cos(np.radians(degrees))]


edges = sharp_edges(box)
# The corners: the vertices of three or more sharp edges.
corners = np.flatnonzero(np.bincount(edges.ravel(), minlength=len(box["V"])) >= 3)
# --8<-- [end:sharp]

# --8<-- [start:features_memory]
# The curves as a mesh on the vertex rows of the box: its edges, each with an id, and its
# points, the fixed vertices, each with an id too.
curves = {
    "V": box["V"],
    "cells": {
        "edge": {"connectivity": edges.astype(np.int64),
                 "attributes": {"edge_id": {"values": np.arange(len(edges), dtype=np.int64)}}},
        "point": {"connectivity": corners.astype(np.int64).reshape(-1, 1),
                  "attributes": {"vertex_id": {"values": corners.astype(np.int64)}}},
    },
}
outputs, _ = wm.isotropic_remeshing(
    {"input": "box", "length_rel": 0.05, "features": "curves"}, {"box": box, "curves": curves})
mesh, feature_edges = outputs["mesh"], outputs["edges"]
edge_id = feature_edges["cells"]["edge"]["attributes"]["edge_id"]["values"]
vertex_id = mesh["vertex_attributes"]["vertex_id"]["values"]
print(f"{len(edges)} sharp edges -> {len(edge_id)} feature edges, "
      f"{len(np.unique(edge_id))} ids, {(vertex_id >= 0).sum()} fixed vertices")
print(f"farthest vertex {farthest_from_box(mesh):.2g} from the box")
# --8<-- [end:features_memory]

# --8<-- [start:features_file]
# The same curves as a features file: indices are vertices of the input file.
with open("box_features.json", "w") as f:
    json.dump({"edges": edges.tolist(),
               "edge_ids": list(range(len(edges))),
               "vertices": [[int(v), int(v)] for v in corners]}, f)
wm.isotropic_remeshing({
    "input": str(DATA / "piece_0.obj"),
    "output": "box_remeshed.vtu",  # and box_remeshed_edge.vtu, the feature edges
    "length_rel": 0.05,
    "features": "box_features.json",
})
# --8<-- [end:features_file]

# --8<-- [start:labels]
# The same curves as the boundaries of a label per face of the box, in a .msh file.
V, F = box["V"], box["cells"]["triangle"]["connectivity"]
normal = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
axis = np.abs(normal).argmax(1)
patch_id = 2 * axis + (normal[np.arange(len(F)), axis] > 0)  # 0 to 5: the side it faces
wm.write_mesh("box_labels.msh", {"V": V, "cells": {"triangle": {
    "connectivity": F, "attributes": {"patch_id": {"values": patch_id.astype(float)}}}}})
wm.isotropic_remeshing({
    "input": "box_labels.msh",
    "output": "box_labels_remeshed.vtu",  # with patch_id on every triangle
    "length_rel": 0.05,
    "feature_fields": ["patch_id"],
})
# --8<-- [end:labels]

# --8<-- [start:label_map]
wm.isotropic_remeshing({
    "input": str(DATA / "topo_annots.msh"),
    "output": "topo_annots_remeshed.vtu",
    "feature_fields": ["tag_0"],
    "length_abs": 2,
    "boundary_eps_rel": 0.001,
    "swap_quality": True,
})
# --8<-- [end:label_map]
