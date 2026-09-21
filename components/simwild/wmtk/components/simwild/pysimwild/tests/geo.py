"""Geometry measurements used by the tests: body-to-body separation,
2D interface roughness/length, and element-inversion checks.

The meshes are read here with gmsh and numpy alone. Nothing is shared with the
implementation under test, so a measurement cannot inherit its defects. The
reader replaced the Python engine's load_mesh, and before it did, the two gave
identical separations, interface edges, roughness, lengths and region nodes on
both fixtures, their irrational-coordinate versions and every mesh a full suite
run wrote (17 meshes, largest relative difference 0)."""
from itertools import combinations

import numpy as np


def read_tagged_msh(msh_path):
    """(dim, coords, cells) of a physical-groups .msh: the node coordinates
    keyed by node tag, and every cell -- its node tags, sorted -- with the set
    of group names it carries (wmtk writes one copy of a cell per group it is
    in, so the copies merge here into one cell)."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.open(str(msh_path))
        dim = 3 if gmsh.model.getPhysicalGroups(3) else 2
        etype = 4 if dim == 3 else 2  # gmsh: 4 = Tet, 2 = Triangle
        node_tags, flat, _ = gmsh.model.mesh.getNodes()
        coords = dict(zip((int(t) for t in node_tags),
                          np.array(flat).reshape(-1, 3)))
        cells = {}
        for d, ptag in gmsh.model.getPhysicalGroups(dim):
            name = gmsh.model.getPhysicalName(d, ptag)
            for ent in gmsh.model.getEntitiesForPhysicalGroup(d, ptag):
                _, ntags = gmsh.model.mesh.getElementsByType(etype, ent)
                for row in np.array(ntags, dtype=np.int64).reshape(-1, dim + 1):
                    key = tuple(sorted(int(v) for v in row))
                    cells.setdefault(key, set()).add(name)
    finally:
        gmsh.finalize()
    return dim, coords, cells


def interface_facets(dim, cells, selection):
    """The facets (triangles in 3D, edges in 2D; sorted node tags) that a
    selection {"region": name, "filter": name} picks: a cell of `region` on one
    side and, on the other, a cell outside it -- one of `filter` when a filter
    is given. The domain boundary has no cell on its other side, so it is never
    picked. Plain group names only; no test here needs an expression."""
    region, filt = selection["region"], selection.get("filter")
    sides = {}
    for cell, names in cells.items():
        for facet in combinations(cell, dim):
            sides.setdefault(facet, []).append(names)
    return [facet for facet, pair in sides.items() if len(pair) == 2 and any(
        region in inside and region not in outside
        and (filt is None or filt in outside)
        for inside, outside in (pair, pair[::-1]))]


def _indexed(coords, facets):
    """Coordinates as an (n,3) array in node-tag order, and the facets as rows
    of indices into it."""
    tags = sorted(coords)
    index = {t: i for i, t in enumerate(tags)}
    V = np.array([coords[t] for t in tags])
    F = np.array([[index[t] for t in f] for f in facets], dtype=np.int64)
    return V, F


def min_separation_3d(msh_path, sel_a, sel_b) -> float:
    """Minimum vertex-to-surface distance between the two selected surfaces
    (checked in both directions). Exact for flat facing surfaces; a tight
    upper bound in general."""
    import igl
    dim, coords, cells = read_tagged_msh(msh_path)
    assert dim == 3, "use interface_polyline_2d for 2D"
    V, Fa = _indexed(coords, interface_facets(dim, cells, sel_a))
    _, Fb = _indexed(coords, interface_facets(dim, cells, sel_b))
    da, _, _ = igl.point_mesh_squared_distance(V[np.unique(Fa)], V, Fb)
    db, _, _ = igl.point_mesh_squared_distance(V[np.unique(Fb)], V, Fa)
    return float(np.sqrt(min(da.min(), db.min())))


def interface_polyline_2d(msh_path, selection):
    """Return (coords (n,2), edges (m,2) indices into coords) of a 2D interface."""
    dim, coords, cells = read_tagged_msh(msh_path)
    assert dim == 2
    V, E = _indexed(coords, interface_facets(dim, cells, selection))
    return V[:, :2], E


def region_node_coords(msh_path, region):
    """{node tag: coordinates} of every node of the cells carrying `region`."""
    _, coords, cells = read_tagged_msh(msh_path)
    tags = {t for cell, names in cells.items() if region in names for t in cell}
    assert tags, f"no cell carries {region!r}"
    return {t: coords[t] for t in sorted(tags)}


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
