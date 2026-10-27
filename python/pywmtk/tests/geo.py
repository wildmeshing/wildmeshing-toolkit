"""Geometry measurements used by the tests: body-to-body separation,
2D interface roughness/length, and element-inversion checks."""
import numpy as np

from simwild.polyfem_ops.constraints import load_mesh


def _split_surfaces(msh_path, sel_a, sel_b):
    """Extract the two selected surfaces; returns (coords, faces_a, faces_b)
    with faces as (n,3) global-index triangle arrays (3D only)."""
    (_, coords, _, _, mesh_dim, faces, _, _, face_tags) = load_mesh(
        str(msh_path),
        selections=[dict(sel_a, id=1), dict(sel_b, id=2)])
    assert mesh_dim == 3, "use interface_edges helpers for 2D"
    F = np.asarray(faces, dtype=np.int64)
    ids = np.array([t[0] for t in face_tags])
    return coords, F[ids == 1], F[ids == 2]


def min_separation_3d(msh_path, sel_a, sel_b) -> float:
    """Minimum vertex-to-surface distance between the two selected surfaces
    (checked in both directions). Exact for flat facing surfaces; a tight
    upper bound in general."""
    import igl
    V, Fa, Fb = _split_surfaces(msh_path, sel_a, sel_b)
    da, _, _ = igl.point_mesh_squared_distance(V[np.unique(Fa)], V, Fb)
    db, _, _ = igl.point_mesh_squared_distance(V[np.unique(Fb)], V, Fa)
    return float(np.sqrt(min(da.min(), db.min())))


def interface_polyline_2d(msh_path, selection):
    """Return (coords (n,2), edges (m,2) global indices) of a 2D interface."""
    (_, coords, edges, _, mesh_dim, _, _, _, _) = load_mesh(
        str(msh_path), selections=[dict(selection, id=1)])
    assert mesh_dim == 2
    return coords, np.asarray(edges, dtype=np.int64)


def roughness_2d(coords, edges) -> float:
    """Total turning of the interface: sum over 2-valent nodes of
    |pi - angle between incident edges|. A straight line scores 0."""
    nbrs: dict = {}
    for a, b in edges:
        nbrs.setdefault(int(a), []).append(int(b))
        nbrs.setdefault(int(b), []).append(int(a))
    total = 0.0
    for n, adj in nbrs.items():
        if len(adj) != 2:
            continue
        u = coords[adj[0]] - coords[n]
        v = coords[adj[1]] - coords[n]
        cosang = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
        ang = np.arccos(np.clip(cosang, -1.0, 1.0))
        total += abs(np.pi - ang)
    return float(total)


def polyline_length(coords, edges) -> float:
    return float(sum(np.linalg.norm(coords[b] - coords[a]) for a, b in edges))


def signed_volumes(msh_path):
    """Signed volumes (3D tets) or signed areas (2D triangles) of every
    primitive, deduplicated across multi-tag copies, in a stable order
    (sorted vertex tuples). Comparing signs pre/post detects inversions."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.open(str(msh_path))
        dim = 3 if gmsh.model.getPhysicalGroups(3) else 2
        etype = 4 if dim == 3 else 2
        npr = 4 if dim == 3 else 3
        node_tags, flat, _ = gmsh.model.mesh.getNodes()
        coords = np.array(flat).reshape(-1, 3)
        idx = {int(t): i for i, t in enumerate(node_tags)}
        prims = {}
        for d, ptag in gmsh.model.getPhysicalGroups(dim):
            for ent in gmsh.model.getEntitiesForPhysicalGroup(d, ptag):
                _, ntags = gmsh.model.mesh.getElementsByType(etype, ent)
                arr = np.array(ntags, dtype=np.int64).reshape(-1, npr)
                for row in arr:
                    prims[tuple(sorted(int(v) for v in row))] = row
    finally:
        gmsh.finalize()

    out = []
    for key in sorted(prims):
        v = coords[[idx[int(t)] for t in prims[key]]]
        if dim == 3:
            out.append(np.linalg.det(np.stack(
                [v[1] - v[0], v[2] - v[0], v[3] - v[0]])) / 6.0)
        else:
            e1, e2 = v[1] - v[0], v[2] - v[0]
            out.append(0.5 * (e1[0] * e2[1] - e1[1] * e2[0]))
    return np.array(out)
