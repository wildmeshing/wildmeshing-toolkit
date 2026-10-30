# docs-example: tested by tests/python/test_tracking_correspondences.py
r"""Coarsen a dense mesh with the Wildmeshing Toolkit's isotropic remeshing inside an envelope,
tetrahedralize the result with tetwild, and keep the correspondence between the dense input
surface and the triangles of the final tet mesh. Shows the meshes and the correspondence in
polyscope.

What it does
------------
1. Reads the input mesh: by default the Stanford bunny, bunny.obj in $WMTK_DATA (data/models,
   from the repository root). Any format the toolkit reads works: OBJ, OFF, STL, MSH, ...
2. Coarsens it with wm.isotropic_remeshing, in memory, with the envelope (eps_rel) and the
   target edge length (length_rel) both relative to the bounding box diagonal.
3. Tetrahedralizes the simplified surface with wm.tetwild, with its own (thinner) envelope and
   edge length, and:
   - skip_simplify: the surface is not simplified again;
   - preserve_topology and check_surface_folds: the surface keeps its topology and does not fold
     over, so the map through tetwild stays a bijection (without them, tens of warnings);
   - filter input: only the tets inside the simplified surface, by its winding number (the bunny
     is open, so neither flood fill nor the tracked surface's winding number apply).
4. Both runs record their history (history_output) and wm.Tracking (the toolkit's tracking
   component) rebuilds each map: one local bijective map per operation, composed in order, with
   warnings where it is not a bijection. tetwild's history starts at the simplified surface
   exactly as given, where the remeshing's ends, so mapping through one map and then the other
   is the map of the two histories concatenated.
5. Maps every dense vertex, as a corner of one of its faces, onto the simplified surface and,
   through both maps, directly onto the tet mesh's tracked triangles: a triangle of the tet mesh
   (by its vertices) and barycentric coordinates in it. Maps the simplified vertices and the tet
   mesh's surface vertices back onto the dense surface.
6. Prints the mesh sizes, timings and map warnings and, per direction, the unmapped vertices and
   the distance from each vertex to its image, relative to the diagonal. Vertices in no face
   (the bunny has about a thousand) are not on the surface and are skipped. Then opens polyscope.

The filter can remove a tracked triangle or two (both its tets are outside); the map records
that as "replaced", and the few dense vertices on them have no image. --tet-filter none keeps
every tet, and every point maps.

Setup
-----
Needs the wildmeshing Python package (see Installation in the documentation) and NumPy; the
viewer also needs polyscope (pip install polyscope). --no-gui runs without it.

Usage
-----
    python tracking_correspondences.py                   # bunny.obj, ~1,080 faces
    python tracking_correspondences.py --eps-rel 1e-2 --length-rel 0.02
    python tracking_correspondences.py path/to/mesh.obj --no-gui

With more than one thread the result changes slightly from run to run (the parallel schedulers'
order is not deterministic); --threads 0 gives identical results.

The viewer
----------
Toggle the structures in the left panel (* shown at start):
    dense input                      the input mesh
    simplified                       the coarse surface mesh
    tet mesh                         the tet mesh (a slice plane shows the inside)
  * tet mesh surface                 its tracked triangles
    dense -> simplified              a line from each dense vertex to its image on the
                                     simplified surface, colored by length
  * dense -> tet surface             the same, onto the tet mesh's tracked triangles
    simplified -> dense              a line from each simplified vertex to its image on the
                                     dense surface
    tet surface -> dense             the same, from the tet mesh's surface vertices
    <any of these>: images           the images, as points
The lines are short, and those ending below a surface are hidden by it: make the mesh
transparent or hide it to see them all. Most are shorter than the envelope, but the map is not a
closest-point projection: a point can also slide along the surface, so some are longer.

How coarse
----------
Face count goes roughly with 1 / length_rel^2, until the envelope stops the collapses. On the
bunny (69,451 faces): eps 1e-2 gives about 4,150 faces at length 0.02, 1,620 at 0.04 and still
1,425 at 0.05; eps 2e-2 gives about 1,200 at length 0.04 and 1,080 at 0.045, the defaults.
tetwild then splits and collapses the surface a little within its envelope: about 1,050-1,130
tracked triangles from the defaults' 1,050 faces.
"""

import argparse
import os
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import wildmeshing as wm

DATA = Path(os.environ.get("WMTK_DATA", "data/models"))
DEFAULT_INPUT = DATA / "bunny.obj"


class HelpFormatter(argparse.RawDescriptionHelpFormatter, argparse.ArgumentDefaultsHelpFormatter):
    """The docstring as written, and the defaults of the options."""


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=HelpFormatter)
    p.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT, help="dense triangle mesh")
    p.add_argument("--eps-rel", type=float, default=2e-2,
                   help="envelope thickness, relative to the bounding box diagonal; the output "
                        "stays within it")
    p.add_argument("--length-rel", type=float, default=0.045,
                   help="target edge length, relative to the bounding box diagonal; larger is "
                        "coarser")
    p.add_argument("--iterations", type=int, default=5,
                   help="remeshing iterations (split, collapse, swap, smooth)")
    p.add_argument("--tet-eps-rel", type=float, default=1e-3,
                   help="tetwild's envelope around the simplified surface, relative to its "
                        "bounding box diagonal")
    p.add_argument("--tet-length-rel", type=float, default=0.05,
                   help="tetwild's target edge length, relative to the simplified surface's "
                        "bounding box diagonal")
    p.add_argument("--tet-filter", choices=["input", "none"], default="input",
                   help="tets tetwild keeps: inside the simplified surface by its winding number, "
                        "or all of them, the bounding box; 'input' can remove a tracked triangle "
                        "or two, whose points then have no image")
    p.add_argument("--threads", type=int, default=8, help="0 runs sequentially")
    p.add_argument("--no-gui", action="store_true", help="compute and print statistics only")
    return p.parse_args(argv)


def corner_points(F, n):
    """One (face, barycentric) per vertex 0..n-1 of F: the vertex as a corner of one of its
    faces. Vertices in no face get face -1."""
    face = np.full(n, -1, dtype=np.int64)
    corner = np.zeros(n, dtype=np.int64)
    for k in range(3):
        face[F[:, k]] = np.arange(len(F))
        corner[F[:, k]] = k
    return face, np.eye(3)[corner]


def corners_of(F, face):
    """The vertices of each face of F; -1 where face is -1."""
    C = F[face]
    C[face < 0] = -1
    return C


def evaluate(V, corners, bary):
    """Positions of points given by their triangle's vertices and barycentric coordinates; NaN
    where the vertices are -1."""
    P = np.full((len(corners), 3), np.nan)
    ok = (corners >= 0).all(axis=1)
    P[ok] = np.einsum("nk,nkd->nd", bary[ok], V[corners[ok]])
    return P


def triangle_ids(T, corners, bary):
    """The row of T with each triple of corners, in any order, and the barycentric coordinates
    reordered as that row's corners; -1 and NaN where there is none."""
    n = max(T.max(), corners.max()) + 1
    weights = np.array([n * n, n, 1], dtype=np.int64)
    keys = np.sort(T, axis=1) @ weights
    query = np.sort(corners, axis=1) @ weights
    order = np.argsort(keys)
    face = order[np.searchsorted(keys, query, sorter=order).clip(max=len(T) - 1)]
    found = (keys[face] == query) & (corners >= 0).all(axis=1)
    # match[i, k, j]: corner k of the row is corner j of the query.
    match = T[face][:, :, None] == corners[:, None, :]
    bary = np.einsum("nkj,nj->nk", match, bary)
    face[~found] = -1
    bary[~found] = np.nan
    return face, bary


def run(args):
    """The meshes, the two maps and the correspondences, as a namespace; prints what it does."""
    r = SimpleNamespace()
    mesh = wm.read_mesh(str(args.input))
    r.V_in = mesh["V"]
    r.F_in = mesh["cells"]["triangle"]["connectivity"]
    r.diag = np.linalg.norm(r.V_in.max(axis=0) - r.V_in.min(axis=0))
    print(f"input {args.input.name}: {len(r.V_in)} vertices, {len(r.F_in)} faces")

    # --8<-- [start:coarsen]
    # 1. Coarsen. On the memory route any non-empty history_output records the history, which
    # comes back as bytes.
    t0 = time.perf_counter()
    outputs, report = wm.isotropic_remeshing({
        "input": "dense",
        "eps_rel": args.eps_rel,
        "length_rel": args.length_rel,
        "max_iterations": args.iterations,
        "num_threads": args.threads,
        "history_output": "history",
    }, {"dense": mesh})
    simplified = outputs["mesh"]
    V_s = simplified["V"]
    F_s = simplified["cells"]["triangle"]["connectivity"]
    remesh_map = wm.Tracking(report["history"])
    # --8<-- [end:coarsen]
    print(f"simplified: {len(V_s)} vertices, {len(F_s)} faces ({time.perf_counter() - t0:.1f} s), "
          f"map warnings: {remesh_map.build_warnings}")
    # --8<-- [start:check_map]
    # Its map's faces are the input's triangles and the output mesh's, as given.
    assert np.array_equal(remesh_map.input_faces, r.F_in)
    assert np.array_equal(remesh_map.output_triangles, F_s)
    # --8<-- [end:check_map]

    # --8<-- [start:tetwild]
    # 2. Tetrahedralize the simplified surface. Its history starts at that surface exactly as
    # given, and ends at the tet mesh's tracked triangles, named by three tet mesh vertices.
    t0 = time.perf_counter()
    outputs, report = wm.tetwild({
        "input": "simplified",
        "eps_rel": args.tet_eps_rel,
        "length_rel": args.tet_length_rel,
        "skip_simplify": True,  # the surface is already as coarse as wanted
        "preserve_topology": True,
        "check_surface_folds": True,
        # The bunny is open: no flood fill, and no winding number of the tracked surface.
        "filter": args.tet_filter,
        "num_threads": args.threads,
        "history_output": "history",
    }, {"simplified": simplified})
    V_t = outputs["mesh"]["V"]
    T_t = outputs["mesh"]["cells"]["tetrahedron"]["connectivity"]
    tet_map = wm.Tracking(report["history"])
    S_t = tet_map.output_triangles
    # --8<-- [end:tetwild]
    print(f"tet mesh: {len(V_t)} vertices, {len(T_t)} tets, {len(S_t)} tracked surface "
          f"triangles ({time.perf_counter() - t0:.1f} s), map warnings: {tet_map.build_warnings}")
    assert np.array_equal(tet_map.input_faces, F_s)

    # --8<-- [start:map_forward]
    # 3. Correspondences. The tet mesh's history starts where the remeshing's ends, so mapping
    # through one map and then the other is the map of the two histories concatenated.
    # Vertices in no face (the bunny has a thousand) are not on the surface and have no image.
    dense_face, bary = corner_points(r.F_in, len(r.V_in))
    s_face, s_bary = remesh_map.to_output(dense_face, bary)
    dense_on_s = evaluate(V_s, corners_of(F_s, s_face), s_bary)
    # Dense vertices directly onto the tet mesh's tracked triangles (rows of S_t).
    t_face, t_bary = triangle_ids(S_t, *tet_map.to_output(s_face, s_bary))
    dense_on_t = evaluate(V_t, corners_of(S_t, t_face), t_bary)
    # --8<-- [end:map_forward]

    # --8<-- [start:map_back]
    # And back: simplified vertices, and the tet mesh's surface vertices, onto the dense surface.
    simplified_face, bary = corner_points(F_s, len(V_s))
    face, bary = remesh_map.to_input(simplified_face, bary)
    s_on_dense = evaluate(r.V_in, corners_of(r.F_in, face), bary)
    tet_face, bary = corner_points(S_t, len(V_t))
    face, bary = remesh_map.to_input(*tet_map.to_input(corners_of(S_t, tet_face), bary))
    t_on_dense = evaluate(r.V_in, corners_of(r.F_in, face), bary)
    # --8<-- [end:map_back]

    print(f"{int((dense_face < 0).sum())} dense vertices in no face, skipped")
    r.V_s, r.F_s, r.V_t, r.T_t, r.S_t = V_s, F_s, V_t, T_t, S_t
    r.remesh_map, r.tet_map = remesh_map, tet_map
    r.dense_face = dense_face
    # name, points, their images, the points that have a face to map from, shown at start
    r.correspondences = [
        ("dense -> simplified", r.V_in, dense_on_s, dense_face >= 0, False),
        ("simplified -> dense", V_s, s_on_dense, simplified_face >= 0, False),
        ("dense -> tet surface", r.V_in, dense_on_t, dense_face >= 0, True),
        ("tet surface -> dense", V_t, t_on_dense, tet_face >= 0, False),
    ]
    # Per direction: the points that should map but have no image, and the distance from each
    # point to its image, relative to the diagonal.
    r.stats = {}
    for name, A, B, mapped, _ in r.correspondences:
        lost = mapped & np.isnan(B).any(axis=1)
        dist = np.linalg.norm(A - B, axis=1)[mapped & ~lost] / r.diag
        r.stats[name] = SimpleNamespace(unmapped=int(lost.sum()), mean=float(dist.mean()),
                                        max=float(dist.max()))
        print(f"{name}: {int(lost.sum())} unmapped, distance / diagonal "
              f"mean {dist.mean():.2e}, max {dist.max():.2e}")
    return r


def correspondence_lines(A, B):
    """Nodes and edges joining A[i] to B[i], for the rows where B is defined."""
    ok = ~np.isnan(B).any(axis=1)
    m = int(ok.sum())
    nodes = np.vstack([A[ok], B[ok]])
    edges = np.stack([np.arange(m), np.arange(m) + m], axis=1)
    return nodes, edges, np.linalg.norm(A[ok] - B[ok], axis=1)


def show(r):
    import polyscope as ps

    ps.init()
    ps.register_surface_mesh("dense input", r.V_in, r.F_in, color=(0.85, 0.85, 0.85),
                             edge_width=0.5, enabled=False)
    ps.register_surface_mesh("simplified", r.V_s, r.F_s, color=(0.35, 0.6, 0.9), edge_width=1.0,
                             enabled=False)
    ps.register_volume_mesh("tet mesh", r.V_t, tets=r.T_t, enabled=False)
    ps.register_surface_mesh("tet mesh surface", r.V_t, r.S_t, color=(0.95, 0.7, 0.3),
                             edge_width=1.0)
    for name, A, B, _, enabled in r.correspondences:
        nodes, edges, dist = correspondence_lines(A, B)
        radius = 0.0004 if A is r.V_in else 0.0008
        lines = ps.register_curve_network(name, nodes, edges, radius=radius, enabled=enabled)
        lines.add_scalar_quantity("distance", dist, defined_on="edges", enabled=True)
        ps.register_point_cloud(f"{name}: images", B[~np.isnan(B).any(axis=1)],
                                radius=2 * radius, color=(0.9, 0.3, 0.2), enabled=False)
    ps.show()


def main():
    args = parse_args()
    r = run(args)
    if not args.no_gui:
        show(r)


if __name__ == "__main__":
    main()
