"""
Export the polyfem interface artifacts from a gmsh .msh:

  1. interface_constraint.hdf5           — fitting constraint anchoring the
                                            interface nodes near rest pose.
  2. interface_constraint_laplacian.hdf5 — Laplacian smoothness constraint
                                            over the interface patch.
  3. interface_collision.obj             — collision proxy mesh (vertices +
                                            edges, plus faces in 3D).
  4. interface_linear_map.hdf5           — displacement map W, shape
                                            [n_proxy_verts, total_n_fe_nodes],
                                            with proxy_disp = W @ fem_disp.
  5. collision_body_ids.txt              — per-proxy-face collision body ids.

Interfaces are picked automatically (every material interface; the CLI path)
or via `selections` — {"region": expr, "filter": expr?, "id": int?} specs
(bare string = region): the boundary of `region`, kept where the outside
satisfies `filter`. Shared with minimum_separation's collision_pairs and
laplacian smoothing's interfaces.

Constraint HDF5 format (polyfem SolveData.cpp):
  local2global        : (n,)       int32  — 0-indexed FE vertex IDs
  A_triplets/rows     : (n,)       int32  — identity row indices
  A_triplets/cols     : (n,)       int32  — identity col indices
  A_triplets/values   : (n,)       float64
  A_triplets/shape    : (2,)       int64  — [n, n]
  b                   : (n, dim)   float64 — zeros

Linear map HDF5 format (polyfem CollisionProxy.cpp):
  weight_triplets          (group, attribute shape=[n_proxy, n_fe_total])
  weight_triplets/rows     : (n,)  int32
  weight_triplets/cols     : (n,)  int32   — input mesh vertex IDs (polyfem remaps internally)
  weight_triplets/values   : (n,)  float64

Usage: python -m simwild.polyfem_ops.constraints sim_input_2.msh
The CLI prints the JSON snippet to add under contact.collision_mesh and
constraints.soft.
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import gmsh
import numpy as np
import h5py
import scipy.sparse

from .mesh_core import (TaggedMesh, assign_selection_ids,
                        select_boundary_faces)


_RED = "\033[31m"
_YELLOW = "\033[33m"
_RESET = "\033[0m"


def _warn(msg: str) -> None:
    print(f"{_YELLOW}WARNING:{_RESET} {msg}", file=sys.stderr)


def _error(msg: str) -> None:
    print(f"{_RED}ERROR:{_RESET} {msg}", file=sys.stderr)
    raise ValueError(msg)


def _orient_edge_loops_2d(
    coords: np.ndarray,
    oriented_edges: list[tuple[int, int]],
) -> list[tuple[int, int]]:
    """Chain directed 2D edges into loops. Each simple closed loop keeps the
    direction its input edges agree on — the explicit region/filter
    orientation from _selected_interfaces, which is the only hole-safe source
    of truth (a cavity loop is wound CW, an outer loop CCW; signed area can't
    tell the material side). A loop whose inputs contradict each other (the
    same interface selected from both sides, so no single direction can be
    correct for both bodies) raises. Non-loop components (open chains, branch
    points) pass through with their original order and orientation."""
    if not oriented_edges:
        return oriented_edges

    # Build undirected adjacency and edge index per component.
    adjacency: dict[int, set[int]] = defaultdict(set)
    comp_edge_indices: dict[int, list[int]] = defaultdict(list)
    for i, (a, b) in enumerate(oriented_edges):
        adjacency[a].add(b)
        adjacency[b].add(a)
        comp_edge_indices[a].append(i)
        comp_edge_indices[b].append(i)

    visited_vertices: set[int] = set()
    result: list[tuple[int, int]] = []

    for seed in adjacency.keys():
        if seed in visited_vertices:
            continue

        # Vertex-connected component.
        stack = [seed]
        comp_vertices: list[int] = []
        while stack:
            v = stack.pop()
            if v in visited_vertices:
                continue
            visited_vertices.add(v)
            comp_vertices.append(v)
            for nb in adjacency[v]:
                if nb not in visited_vertices:
                    stack.append(nb)

        comp_vertex_set = set(comp_vertices)
        comp_edges_idx = set()
        for v in comp_vertices:
            comp_edges_idx.update(comp_edge_indices[v])

        degrees = {v: sum(1 for nb in adjacency[v] if nb in comp_vertex_set) for v in comp_vertices}
        is_simple_loop = len(comp_edges_idx) >= 3 and all(deg == 2 for deg in degrees.values())

        if not is_simple_loop:
            # Preserve original orientation/order for non-loop components.
            for i, e in enumerate(oriented_edges):
                if i in comp_edges_idx:
                    result.append(e)
            continue

        # Reconstruct loop order (deterministic seed/neighbor choice).
        start = min(comp_vertices)
        n0, n1 = sorted(adjacency[start])
        curr = n0
        prev = start
        loop = [start, curr]
        while curr != start:
            neigh = sorted(adjacency[curr])
            nxt = neigh[0] if neigh[0] != prev else neigh[1]
            prev, curr = curr, nxt
            if curr != start:
                loop.append(curr)

        directed = {oriented_edges[i] for i in comp_edges_idx}
        agree = sum((loop[i], loop[(i + 1) % len(loop)]) in directed
                    for i in range(len(loop)))
        disagree = sum((loop[(i + 1) % len(loop)], loop[i]) in directed
                       for i in range(len(loop)))
        if agree and disagree:
            _error(
                f"interface loop through vertex {min(comp_vertices)} has "
                f"contradictory orientations ({agree} edges one way, "
                f"{disagree} the other): the same interface was selected "
                f"from both sides, so no single orientation is correct — "
                f"drop one side of the selection")
        if disagree:
            loop = loop[::-1]

        for i in range(len(loop)):
            a = loop[i]
            b = loop[(i + 1) % len(loop)]
            result.append((a, b))

    return result
# ---------------------------------------------------------------------------
# Interface selection (pair expressions; shared core in mesh_core)
# ---------------------------------------------------------------------------

def _selected_interfaces(mesh: TaggedMesh, selections: list):
    """Selection-scoped interfaces: returns (oriented_faces_3d,
    oriented_edges_2d, tags_rows). Orientation: 3D normals point out of the
    region; 2D left-normals point toward the region (legacy GCP convention);
    the downstream loop pass preserves this orientation and only dedupes."""
    try:
        unique, _ = assign_selection_ids(selections)
        records = select_boundary_faces(mesh, unique)
    except ValueError as e:
        _error(str(e))
    faces3d, edges2d, tags = [], [], []
    for r in records:
        inside = mesh.centroid(r["a_prim"])
        outside = mesh.centroid(r["b_prim"])
        if mesh.mesh_dim == 3:
            a, b, c = r["face"]
            n = np.cross(mesh.coords[b] - mesh.coords[a],
                         mesh.coords[c] - mesh.coords[a])
            faces3d.append((a, c, b) if np.dot(n, outside - inside) < 0
                           else (a, b, c))
        else:
            a, b = r["face"]
            t = mesh.coords[b] - mesh.coords[a]
            left = np.array([-t[1], t[0]])
            edges2d.append((b, a) if np.dot(left, inside - outside) < 0
                           else (a, b))
        tags.append(r["ids"])
    return faces3d, edges2d, tags


def _auto_interfaces(mesh: TaggedMesh):
    """Legacy auto-detect: every face whose incident phys-tag multiset is not
    a same-tag interior pair — material interfaces plus one-sided boundary
    skins. Orientation: hi phys tag = inside (3D normals outward from it)."""
    faces3d, edges2d, tags = [], [], []
    for fk, prims in mesh.face_to_prims.items():
        incidents = [(mesh.names[n], mesh.centroid(p))
                     for p in prims for n in sorted(mesh.prim_tags[p])]
        ptags = [t for t, _ in incidents]
        unique = set(ptags)
        if len(ptags) == 2 and len(unique) == 1:
            continue
        if len(ptags) == 1 and ptags[0] == 0:
            continue
        fr = mesh.face_repr[fk]
        if len(incidents) == 1:
            inside = incidents[0][1]
            mid = np.mean([mesh.coords[v] for v in fr], axis=0)
            outward = mid - inside
        else:
            hi, lo = max(unique), min(unique)
            hi_c = np.mean([c for t, c in incidents if t == hi], axis=0)
            lo_c = np.mean([c for t, c in incidents if t == lo], axis=0)
            outward = lo_c - hi_c
        if mesh.mesh_dim == 3:
            a, b, c = fr
            n = np.cross(mesh.coords[b] - mesh.coords[a],
                         mesh.coords[c] - mesh.coords[a])
            faces3d.append((a, c, b) if np.dot(n, outward) < 0 else (a, b, c))
        else:
            a, b = fr
            t = mesh.coords[b] - mesh.coords[a]
            left = np.array([-t[1], t[0]])
            edges2d.append((b, a) if np.dot(left, -outward) < 0 else (a, b))
        tags.append(sorted(unique))
    return faces3d, edges2d, tags


def load_mesh(msh_path: str, selections: list | None = None):
    """Load a multi-tag .msh and extract its material-interface surfaces —
    every interface when `selections` is omitted, else those selected by the
    given {"region", "filter", "id"} specs (see mesh_core). Returns the 9-tuple
    (node_tag_to_idx, coords (n, mesh_dim) float64 in mesh units,
    interface_edges, total_n_nodes, mesh_dim, interface_faces (3D, else []),
    collision_node_ids in collision-OBJ vertex order, collision_edges_local
    over those local IDs, collision_face_tags — per-face sorted id lists
    (interface ids when selected, phys tags in auto mode))."""
    mesh = TaggedMesh(msh_path)
    if selections:
        faces3d, edges2d, face_tags = _selected_interfaces(mesh, selections)
    else:
        faces3d, edges2d, face_tags = _auto_interfaces(mesh)

    collision_node_ids: list = []
    collision_edges_local: list = []

    if mesh.mesh_dim == 3:
        interface_faces = faces3d
        edge_set = set()
        for a, b, c in interface_faces:
            edge_set.add((min(a, b), max(a, b)))
            edge_set.add((min(b, c), max(b, c)))
            edge_set.add((min(c, a), max(c, a)))
        interface_edges = sorted(edge_set)
        if interface_faces:
            collision_node_ids = sorted({v for f in interface_faces for v in f})
            g2l = {g: i for i, g in enumerate(collision_node_ids)}
            collision_edges_local = [(g2l[a], g2l[b]) for a, b in interface_edges]
    else:
        # The loop pass preserves the explicit per-edge orientation and
        # raises on contradictions; union id rows per undirected edge
        # (multi-interface edges keep every id).
        edge_to_tags: dict = {}
        for (a, b), t in zip(edges2d, face_tags):
            key = (min(a, b), max(a, b))
            edge_to_tags[key] = (sorted(set(edge_to_tags[key]) | set(t))
                                 if key in edge_to_tags else t)
        interface_edges = _orient_edge_loops_2d(mesh.coords, edges2d)
        face_tags = [edge_to_tags[(min(a, b), max(a, b))]
                     for a, b in interface_edges]
        interface_faces = []
        if interface_edges:
            collision_node_ids = sorted({v for e in interface_edges for v in e})
            g2l = {g: i for i, g in enumerate(collision_node_ids)}
            collision_edges_local = [(g2l[a], g2l[b]) for a, b in interface_edges]

    return (
        mesh.node_tag_to_idx,
        mesh.coords,
        interface_edges,
        mesh.total_n_nodes,
        mesh.mesh_dim,
        interface_faces,
        collision_node_ids,
        collision_edges_local,
        face_tags,
    )


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------

def write_fitting_constraint_hdf5(path: str, node_ids: np.ndarray, dim: int = 2, coords: np.ndarray = None, interface_edges: list = None, graph: bool = False, normalize: bool = False, interface_faces: list = None) -> None:
    """Write the fitting constraint HDF5 (local2global, A_triplets, b = 0)
    with A = sqrt(M), so (1/2)||A u||^2 is a proper interface L2 norm;
    `normalize` also divides A by sqrt(L_total) so a uniform displacement
    u_ref costs w * u_ref^2."""
    n = len(node_ids)
    rows, cols, values = get_mass_matrix(coords, interface_edges, node_ids, graph, interface_faces=interface_faces)
    if normalize:
        L_total = np.sum(values)
        values = np.sqrt(values) / np.sqrt(L_total)
        print(f"  fit norm   : L_total = {L_total:.6g} (mesh units)")
    else:
        values = np.sqrt(values)

    with h5py.File(path, "w") as f:
        f.create_dataset("local2global", data=node_ids)
        grp = f.create_group("A_triplets")
        grp.create_dataset("rows",   data=rows)
        grp.create_dataset("cols",   data=cols)
        grp.create_dataset("values", data=values)
        grp.create_dataset("shape",  data=np.array([n, n], dtype=np.int64))
        f.create_dataset("b", data=np.zeros((n, dim), dtype=np.float64))

def write_pin_constraint_hdf5(path: str, node_ids: np.ndarray, dim: int) -> None:
    """Write a pin constraint (A = I on node_ids, b = 0): as a polyfem HARD
    constraint (constraints/hard) it holds those nodes exactly at rest via
    the augmented Lagrangian — used for min-sep protected_regions."""
    n = len(node_ids)
    with h5py.File(path, "w") as f:
        # h5pp reads local2global as vector<int> — must be int32 on disk
        f.create_dataset("local2global", data=np.asarray(node_ids, dtype=np.int32))
        grp = f.create_group("A_triplets")
        grp.create_dataset("rows",   data=np.arange(n, dtype=np.int32))
        grp.create_dataset("cols",   data=np.arange(n, dtype=np.int32))
        grp.create_dataset("values", data=np.ones(n, dtype=np.float64))
        grp.create_dataset("shape",  data=np.array([n, n], dtype=np.int64))
        f.create_dataset("b", data=np.zeros((n, dim), dtype=np.float64))
    print(f"  pinned     : {path}  ({n} nodes)")


def get_mass_matrix(coords: np.ndarray, interface_edges: list, node_ids: np.ndarray, graph: bool,
                    interface_faces: list = None) -> np.ndarray:
    """Lumped mass matrix over the interface as diagonal COO triplets, indexed
    locally (local index = position in node_ids). 3D: igl barycentric mass
    (area-weighted); 2D: half-edge-length lumping; graph: identity."""
    n = len(node_ids)
    global_to_local = {int(gid): lid for lid, gid in enumerate(node_ids)}

    if graph:
        return (np.arange(n, dtype=np.int32),
                np.arange(n, dtype=np.int32),
                np.ones(n, dtype=np.float64))

    if interface_faces:
        import igl
        # Build local V and F arrays over collision_node_ids
        V = coords[node_ids]  # (n, 3)
        F = np.array([[global_to_local[a], global_to_local[b], global_to_local[c]]
                      for a, b, c in interface_faces], dtype=np.int32)
        M = igl.massmatrix(V, F, igl.MASSMATRIX_TYPE_BARYCENTRIC)
        diag = np.array(M.diagonal(), dtype=np.float64)
        rows = np.arange(n, dtype=np.int32)
        cols = np.arange(n, dtype=np.int32)
        return rows, cols, diag

    vertex_mass = np.zeros(n, dtype=np.float64)
    for a, b in interface_edges:
        la, lb = global_to_local.get(a), global_to_local.get(b)
        if la is None or lb is None:
            continue
        edge_length = np.linalg.norm(coords[a] - coords[b])
        vertex_mass[la] += edge_length / 2.0
        vertex_mass[lb] += edge_length / 2.0

    rows = np.arange(n, dtype=np.int32)
    cols = np.arange(n, dtype=np.int32)
    return rows, cols, vertex_mass

def get_stiffness_matrix(coords: np.ndarray, interface_edges: list, node_ids: np.ndarray, graph: bool,
                          interface_faces: list = None) -> np.ndarray:
    """Stiffness matrix S over the interface as COO triplets (zero row sums,
    off-diagonal <= 0, diagonal >= 0), indexed locally like get_mass_matrix.
    3D: negated igl.cotmatrix; 2D: 1/edge-length weights; graph: unit weights."""
    global_to_local = {int(gid): lid for lid, gid in enumerate(node_ids)}

    if not graph and interface_faces:
        import igl
        V = coords[node_ids]  # (n, 3)
        F = np.array([[global_to_local[a], global_to_local[b], global_to_local[c]]
                      for a, b, c in interface_faces], dtype=np.int32)
        # igl.cotmatrix: off-diagonal >= 0, diagonal <= 0 (negative semi-definite Laplacian).
        # Negate to match our convention: off-diagonal <= 0, diagonal >= 0.
        S = -igl.cotmatrix(V, F)
        S_coo = S.tocoo()
        return S_coo.row.astype(np.int32), S_coo.col.astype(np.int32), S_coo.data.astype(np.float64)

    stiffness = defaultdict(float)
    for a, b in interface_edges:
        la, lb = global_to_local.get(a), global_to_local.get(b)
        if la is None or lb is None:
            continue
        edge_length = np.linalg.norm(coords[a] - coords[b])
        weight = 1.0 if graph else 1.0 / edge_length
        stiffness[(la, lb)] -= weight
        stiffness[(lb, la)] -= weight
        stiffness[(la, la)] += weight
        stiffness[(lb, lb)] += weight

    rows = np.array([k[0] for k in stiffness.keys()], dtype=np.int32)
    cols = np.array([k[1] for k in stiffness.keys()], dtype=np.int32)
    values = np.array(list(stiffness.values()), dtype=np.float64)
    return rows, cols, values

def get_laplacian_matrix(coords: np.ndarray, interface_edges: list, node_ids: np.ndarray, graph: bool,
                          interface_faces: list = None) -> np.ndarray:
    """Laplacian L = M^{-1} S over the interface as COO triplets: the
    get_stiffness_matrix triplets scaled by the inverse lumped mass diagonal,
    indexed locally like get_mass_matrix."""
    S_rows, S_cols, S_values = get_stiffness_matrix(coords, interface_edges, node_ids, graph, interface_faces=interface_faces)
    _, _, M_values = get_mass_matrix(coords, interface_edges, node_ids, graph, interface_faces=interface_faces)

    # M is diagonal; M_values[i] is the i-th local diagonal entry
    M_inv = 1.0 / M_values  # shape (n,)

    L_values = M_inv[S_rows] * S_values
    return S_rows.copy(), S_cols.copy(), L_values

def write_laplacian_constraint_hdf5(path: str, node_ids: np.ndarray, coords: np.ndarray, interface_edges: list, graph: bool = False, scale: float = 1.0, normalize: bool = False, interface_faces: list = None, smooth_positions: bool = False) -> None:
    """Write the Laplacian smoothness constraint HDF5 with A = L. Default
    b = 0 enforces harmonic displacements (L·u = 0; zero energy and gradient
    at rest, preserving equilibrium); smooth_positions instead sets
    b = -L·(scale * rest_coords) so absolute positions are harmonic (scale =
    solver units per mesh unit). `normalize` divides A by ||L||_F for a
    mesh-independent, O(1) energy on smooth u."""
    n = len(node_ids)
    dim = coords.shape[1]
    # Check zero-row-sum on the stiffness matrix S, where the property is
    # constructed exactly. L = M^{-1} S also has zero row sums in exact
    # arithmetic, but if M has wide dynamic range (small triangles → big
    # M_inv), float noise in S's row sums gets amplified past any fixed
    # absolute threshold on L. Asserting on S is the honest check.
    S_rows, S_cols, S_values = get_stiffness_matrix(coords, interface_edges, node_ids, graph, interface_faces=interface_faces)
    S = scipy.sparse.coo_matrix((S_values, (S_rows, S_cols)), shape=(n, n)).tocsr()
    S_row_sums = np.array(S.sum(axis=1)).flatten()
    S_scale = max(np.abs(S_values).max() if len(S_values) else 1.0, 1.0)
    assert np.all(np.abs(S_row_sums) < 1e-10 * S_scale), \
        f"S should have zero row sums (max |row_sum| = {np.abs(S_row_sums).max():.3e}, scale = {S_scale:.3e})"

    L_rows, L_cols, L_values = get_laplacian_matrix(coords, interface_edges, node_ids, graph, interface_faces=interface_faces)
    if normalize:
        L_frob = np.sqrt(np.sum(L_values ** 2))
        L_values = L_values / L_frob
        print(f"  lap norm   : ||L||_F = {L_frob:.6g} (mesh units)")
    L = scipy.sparse.coo_matrix((L_values, (L_rows, L_cols)), shape=(n, n)).tocsr()

    if smooth_positions:
        X = scale * coords[node_ids]
        b = -L @ X  # (n, dim)
    else:
        b = np.zeros((n, dim), dtype=np.float64)

    with h5py.File(path, "w") as f:
        f.create_dataset("local2global", data=node_ids)
        grp = f.create_group("A_triplets")
        grp.create_dataset("rows",   data=L_rows)
        grp.create_dataset("cols",   data=L_cols)
        grp.create_dataset("values", data=L_values)
        grp.create_dataset("shape",  data=np.array([n, n], dtype=np.int64))
        f.create_dataset("b", data=b)

def write_collision_mesh_obj(path: str, coords: np.ndarray,
                             node_ids: np.ndarray,
                             interface_edges: list,
                             interface_faces: list | None = None,
                             collision_node_ids: list[int] | None = None,
                             collision_edges_local: list[tuple[int, int]] | None = None) -> None:
    """Write the collision proxy OBJ: vertices plus 'f' faces (3D) or 'l'
    edges (2D), falling back to node_ids/interface_edges when the collision_*
    args are empty. Vertices are in mesh units (un-scaled); polyfem applies
    the geometry transformation from the JSON."""
    # Map global node ID → local index in the collision mesh (1-based for OBJ)
    if collision_node_ids is None or len(collision_node_ids) == 0:
        collision_node_ids = list(node_ids)
    idx_map = {nid: i + 1 for i, nid in enumerate(collision_node_ids)}

    with open(path, "w") as f:
        f.write("# Interface collision mesh\n")
        for nid in collision_node_ids:
            if coords.shape[1] == 2:
                x, y = coords[nid]
                z = 0.0
            else:
                x, y, z = coords[nid]
            f.write(f"v {x} {y} {z}\n")

        if interface_faces:
            for a, b, c in interface_faces:
                if a in idx_map and b in idx_map and c in idx_map:
                    f.write(f"f {idx_map[a]} {idx_map[b]} {idx_map[c]}\n")
            print(f"  collision  : {path}  ({len(collision_node_ids)} verts, {len(interface_faces)} faces)")
        else:
            if collision_edges_local:
                for a, b in collision_edges_local:
                    f.write(f"l {a + 1} {b + 1}\n")
                n_edges_out = len(collision_edges_local)
            else:
                n_edges_out = 0
                for a, b in interface_edges:
                    if a in idx_map and b in idx_map:
                        f.write(f"l {idx_map[a]} {idx_map[b]}\n")
                        n_edges_out += 1
            print(f"  collision  : {path}  ({len(collision_node_ids)} verts, {n_edges_out} edges)")


def write_linear_map_hdf5(path: str, node_ids: np.ndarray,
                          total_n_nodes: int) -> None:
    """Write the identity displacement map proxy_vert[i] ← fe_node[node_ids[i]]
    as a weight_triplets HDF5 group with shape attribute
    [n_proxy, total_n_nodes], per polyfem CollisionProxy.cpp."""
    n = len(node_ids)
    rows   = np.arange(n, dtype=np.int32)
    cols   = node_ids.astype(np.int32)   # input mesh vertex IDs (polyfem remaps internally)
    values = np.ones(n, dtype=np.float64)
    shape  = np.array([n, total_n_nodes], dtype=np.int64)

    with h5py.File(path, "w") as f:
        grp = f.create_group("weight_triplets")
        grp.attrs["shape"] = shape          # attribute, not dataset
        grp.create_dataset("rows",   data=rows)
        grp.create_dataset("cols",   data=cols)
        grp.create_dataset("values", data=values)

    print(f"  linear map : {path}  (shape {shape.tolist()})")


def write_collision_body_ids_txt(path: str, face_tags: list[list[int]]) -> None:
    """Write one space-separated line of collision body IDs per face/edge,
    row order matching the OBJ."""
    with open(path, "w") as f:
        for tags in face_tags:
            f.write(" ".join(str(t) for t in tags) + "\n")
    print(f"  body IDs   : {path}  ({len(face_tags)} faces)")


# ---------------------------------------------------------------------------
# Callable entry point (importable from Python)
# ---------------------------------------------------------------------------

def make_interface_constraint(
    mesh_path: str,
    selections: list | None = None,
    out_dir: str | None = None,
    constraint_path: str | None = None,
    collision_path: str | None = None,
    linear_map_path: str | None = None,
    body_ids_path: str | None = None,
    use_graph: bool = False,
    normalize: bool = False,
    scale: float = 0.001,
    smooth_positions: bool = False,
    dim: int | None = None,
    skip_collision_artifacts: bool = False,
) -> None:
    """Extract interface surfaces from a multi-tag .msh (`selections` as in
    load_mesh; omitted = every material interface) and write the polyfem
    artifacts, by default next to the mesh or under out_dir:
      interface_constraint.hdf5 / interface_constraint_laplacian.hdf5,
      interface_collision.obj (collision proxy / selection viz), and — unless
      skip_collision_artifacts (smoothing mode; polyfem doesn't read them) —
      interface_linear_map.hdf5 and collision_body_ids.txt.
    """
    mp = Path(mesh_path)
    od = Path(out_dir) if out_dir else (mp.parent if mp.parent != Path("") else Path("."))
    constraint_path = constraint_path or str(od / "interface_constraint.hdf5")
    collision_path  = collision_path  or str(od / "interface_collision.obj")
    linear_map_path = linear_map_path or str(od / "interface_linear_map.hdf5")
    body_ids_path   = body_ids_path   or str(od / "collision_body_ids.txt")

    print(f"Reading {mesh_path} ...")
    (
        _,
        coords,
        interface_edges,
        total_n_nodes,
        mesh_dim,
        interface_faces,
        collision_node_ids,
        collision_edges_local,
        collision_face_tags,
    ) = load_mesh(str(mesh_path), selections=selections)

    if dim is None:
        dim = mesh_dim

    if not interface_edges:
        _error("no interface edges extracted — check selections and mesh physical groups")

    node_ids = np.array(sorted({v for e in interface_edges for v in e}), dtype=np.int32)
    print(f"Found {len(interface_edges)} interface edges, {len(node_ids)} interface nodes")
    print("Writing:")

    write_fitting_constraint_hdf5(
        constraint_path, node_ids, dim=dim, coords=coords,
        interface_edges=interface_edges, graph=use_graph, normalize=normalize,
        interface_faces=interface_faces if interface_faces else None,
    )
    write_laplacian_constraint_hdf5(
        constraint_path.replace(".hdf5", "_laplacian.hdf5"),
        node_ids, coords, interface_edges, graph=use_graph, scale=scale,
        normalize=normalize,
        interface_faces=interface_faces if interface_faces else None,
        smooth_positions=smooth_positions,
    )
    # Always write the OBJ — it's tiny and the only convenient way to
    # visualize which faces the selection rule actually picked. Polyfem
    # doesn't read it in smoothing mode but it doesn't hurt to have.
    write_collision_mesh_obj(
        collision_path, coords, node_ids, interface_edges,
        interface_faces=interface_faces,
        collision_node_ids=collision_node_ids,
        collision_edges_local=collision_edges_local,
    )
    if not skip_collision_artifacts:
        map_node_ids = np.array(collision_node_ids if collision_node_ids else node_ids, dtype=np.int32)
        write_linear_map_hdf5(linear_map_path, map_node_ids, total_n_nodes)
        write_collision_body_ids_txt(body_ids_path, collision_face_tags)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh", help="Input .msh file (polyfem sim mesh)")
    ap.add_argument("--dim", type=int, default=None, help="Spatial dimension for constraints (default: auto-detect from mesh)")
    ap.add_argument("--constraint",  default=None,
                    help="Output constraint HDF5 path (default: input mesh folder/interface_constraint.hdf5)")
    ap.add_argument("--collision",   default=None,
                    help="Output collision OBJ path (default: input mesh folder/interface_collision.obj)")
    ap.add_argument("--linear-map",  default=None,
                    help="Output linear-map HDF5 path (default: input mesh folder/interface_linear_map.hdf5)")
    ap.add_argument("--graph", action="store_true", help="Use unweighted graph Laplacian instead of FEM Laplacian for constraints")
    ap.add_argument("--normalize", action="store_true",
                    help="Normalize constraint matrices: fitting by 1/sqrt(L_total), Laplacian by 1/||L||_F")
    ap.add_argument("--scale", type=float, default=0.001,
                    help="Scale factor applied to mesh coordinates when computing Laplacian RHS (e.g. 0.001 to convert mm to m)")
    ap.add_argument("--body-ids", default=None,
                    help="Output collision body IDs txt path (default: input mesh folder/collision_body_ids.txt)")
    ap.add_argument("--smooth-positions", action="store_true",
                    help="Laplacian smooths absolute positions (L·x=0) instead of displacements (L·u=0, default)")
    args = ap.parse_args()

    mesh_path = Path(args.mesh)
    out_dir = mesh_path.parent if mesh_path.parent != Path("") else Path(".")
    constraint_path  = args.constraint  or str(out_dir / "interface_constraint.hdf5")
    collision_path   = args.collision   or str(out_dir / "interface_collision.obj")
    linear_map_path  = args.linear_map  or str(out_dir / "interface_linear_map.hdf5")
    body_ids_path    = args.body_ids    or str(out_dir / "collision_body_ids.txt")

    make_interface_constraint(
        mesh_path=str(mesh_path),
        selections=None,   # CLI: legacy auto-detect (no explicit selections)
        out_dir=str(out_dir),
        constraint_path=constraint_path,
        collision_path=collision_path,
        linear_map_path=linear_map_path,
        body_ids_path=body_ids_path,
        use_graph=args.graph,
        normalize=args.normalize,
        scale=args.scale,
        smooth_positions=args.smooth_positions,
        dim=args.dim,
    )

    print("\nAdd to your sim JSON:")
    body_ids_snippet = f'\n          "collision_body_ids": "{body_ids_path}",'
    print(f"""  "contact": {{
      "collision_mesh": {{
          "enabled": true,
          "mesh": "{collision_path}",
          "linear_map": "{linear_map_path}"{body_ids_snippet}
      }}
  }},
  "constraints": {{
      "soft": [{{"data": "{constraint_path}", "weight": 1e6}}]
  }}""")


if __name__ == "__main__":
    main()
