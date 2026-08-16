#!/usr/bin/env python3
"""Polyscope viewer for a topological_offset result: region surfaces + distance error.

    ./visualize_offset.py runs/prism                 # a run directory
    ./visualize_offset.py out.msh                    # mesh alone
    ./visualize_offset.py out.msh config.json        # mesh + the config it ran under

Needs polyscope, meshio and numpy. The simwild conda env has all three:

    conda activate simwild && python visualize_offset.py <run dir>

Otherwise any env with those packages works (see the triwild scripts README for a venv).

The output .msh holds one cell block per physical group -- `ambient`, one group per tag,
and the offset output tag(s). Nothing in the file marks the surfaces the optimization is
about, so they are derived here the way the C++ derives them from the labels: an
interface face (3D) or edge (2D) is one whose two incident cells lie in different
groups, and

  input surface     boundary of the non-ambient, non-offset groups -- the complex the
                    offset is measured against (orange)
  offset surface    offset group against ambient: the band's OUTER surface, the one
                    supposed to sit at target_distance. Coloured by |dist-delta|/delta
                    by default (reds, white 0 -> dark red at the max, range shown in the
                    panel); switchable to raw distance (viridis) or solid blue
  inner interface   offset group against the input groups: hugs the complex at distance
                    ~0 by construction, drawn dim and off by default, so it cannot be
                    mistaken for the offset
  region boundaries every facet where the two sides' tag memberships differ (green);
                    on by default when the input-surface layer is empty
  envelope curves   the EnvelopeSurface line entity, 2D outputs only: the
                    region-boundary geometry the envelope was built from (orange)

The offset surface carries two scalar layers: distance to the input surface, and
|distance - delta| / delta. delta comes from the config (`target_distance`, else
`target_distance_rel` x the mesh bounding-box diagonal -- the box is the frozen
background bbox, so the output diagonal equals the input one). Without a config the
defaults (rel 0.01, tag "offset") are assumed and said so.

Two honesty notes on the error layer:

* The reference surface is the band's INNER interface (which hugs the input complex by
  construction, whatever the selection expression was), not the complex the C++ BVH
  holds -- they differ where the band does not fully wrap the complex. Distances to it
  are exact point-to-facet.
* It is sampled at offset VERTICES, like compute_distance_deviation() -- so it inherits
  that metric's known blind spot between vertices.
"""

import json
import re
import sys
from pathlib import Path

import meshio
import numpy as np
import polyscope as ps
import polyscope.imgui as psim

C_INPUT = (0.910, 0.525, 0.165)  # orange/blue: separable under colour vision deficiency
C_OFFSET = (0.169, 0.498, 0.831)
C_INNER = (0.55, 0.55, 0.58)
C_OTHER = (0.55, 0.75, 0.55)


def eval_selection(expr, names):
    """The offset_selection boolean expression, on one cell's group memberships.

    Same language the C++ parses: tag names, & | !, parentheses; 'ambient' (spec default
    spells it '_' or '!_') means "in no tag group". Names are substituted with their
    membership truth value, then the expression is evaluated -- nothing but booleans and
    the three operators survives the substitution, so eval sees no cell data.
    """
    import re

    def sub(m):
        tok = m.group(0)
        if tok in ("ambient", "_"):
            return str(names == {"ambient"} or not (names - {"ambient"}))
        return str(tok in names)

    py = re.sub(r"[A-Za-z0-9_]+", sub, expr)  # tag names may be purely numeric
    py = py.replace("!", " not ").replace("&", " and ").replace("|", " or ")
    if not re.fullmatch(r"[ ()TrueFalsandnot]*", py):
        raise ValueError("cannot evaluate offset_selection %r" % expr)
    return bool(eval(py))


def read_groups(path):
    """(points, dim, {group name: cell array}, envelope segments) from the .msh."""
    m = meshio.read(str(path))
    kind, ncol = ("tetra", 4) if any(c.type == "tetra" for c in m.cells) else ("triangle", 3)
    # One block per physical group, in the same order as field_data; pair them by the
    # physical id gmsh stamped on each block's cells.
    id_to_name = {int(v[0]): k for k, v in m.field_data.items() if int(v[1]) == (kind == "tetra") + 2}
    groups = {}
    for block, phys in zip(m.cells, m.cell_data["gmsh:physical"]):
        if block.type != kind or len(block.data) == 0:
            continue
        name = id_to_name.get(int(phys[0]), "group_%d" % int(phys[0]))
        groups.setdefault(name, []).append(block.data)
    groups = {k: np.vstack(v) for k, v in groups.items()}
    # The 2D writer also emits the envelope's segments as an "EnvelopeSurface" line
    # entity -- the region-boundary geometry the envelope was built from, which is the
    # closest thing in the file to the input geometry itself. (The 3D writer's envelope
    # block is commented out, so this is empty for tets.)
    env = [c.data for c in m.cells if c.type == "line"]
    env = np.vstack(env) if env else np.zeros((0, 2), np.int64)
    return m.points, ncol - 1, groups, env


def read_groups_vtu(path):
    """(points, dim, {group name: cell array}, envelope segments) from a debug .vtu frame.

    The debug frames the optimization writes are .vtu, not .msh, and carry their groups
    differently: write_vtu() emits one CELL FIELD per tag (1 where the cell carries it) plus
    `offset_tag`, which is 1 on the band and is derived from the construction LABEL rather than
    from the tags -- so it is the reliable band marker even where a tag was written elsewhere.
    """
    m = meshio.read(str(path))
    kind, ncol = ("tetra", 4) if any(c.type == "tetra" for c in m.cells) else ("triangle", 3)
    cells = np.vstack([c.data for c in m.cells if c.type == kind])
    groups = {}
    for name, arrays in m.cell_data.items():
        vals = np.concatenate([
            np.asarray(a).reshape(-1)
            for a, c in zip(arrays, m.cells) if c.type == kind
        ])
        sel = cells[vals > 0.5]
        if len(sel):
            groups[name] = sel
    env = [c.data for c in m.cells if c.type == "line"]
    env = np.vstack(env) if env else np.zeros((0, 2), np.int64)
    return m.points, ncol - 1, groups, env


def read_any(path):
    return read_groups_vtu(path) if path.suffix == ".vtu" else read_groups(path)


def interfaces(points, dim, groups):
    """{(group a, group b): facet array}, a < b, plus (group, '') for domain boundary."""
    names = sorted(groups)
    cells = np.vstack([groups[n] for n in names])
    owner = np.concatenate([np.full(len(groups[n]), i) for i, n in enumerate(names)])

    # Every facet of every cell, keyed by its sorted vertex tuple.
    nf = dim + 1  # facets per cell: 4 faces of a tet, 3 edges of a triangle
    keep = [[j for j in range(nf) if j != skip] for skip in range(nf)]
    facets = np.concatenate([cells[:, k] for k in keep])  # (nf * ncells, dim)
    facet_owner = np.tile(owner, nf)
    key = np.sort(facets, axis=1)

    order = np.lexsort(key.T[::-1])
    key, facets, facet_owner = key[order], facets[order], facet_owner[order]
    new = np.ones(len(key), bool)
    new[1:] = (key[1:] != key[:-1]).any(axis=1)
    starts = np.flatnonzero(new)
    counts = np.diff(np.append(starts, len(key)))

    out = {}
    for s, c in zip(starts, counts):
        if c == 1:
            pair = (names[facet_owner[s]], "")  # domain boundary
        elif facet_owner[s] != facet_owner[s + 1]:
            a, b = sorted((names[facet_owner[s]], names[facet_owner[s + 1]]))
            pair = (a, b)
        else:
            continue  # interior to one group
        out.setdefault(pair, []).append(facets[s])
    return {k: np.asarray(v) for k, v in out.items()}


def compact(points, cells):
    used = np.unique(cells)
    remap = np.full(len(points), -1, np.int64)
    remap[used] = np.arange(len(used))
    return points[used], remap[cells]


def sample_regular_grid(pts, vals, q):
    """Bilinear sample of a field defined on the regular grid `pts` (as written by
    write_phi_grid) at the query points `q`. None if `pts` is not such a grid."""
    n = int(round(np.sqrt(len(pts))))
    if n < 2 or n * n != len(pts):
        return None
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = np.where(hi - lo > 0, hi - lo, 1.0)
    g = np.asarray(vals).reshape(n, n)  # sample k = j*n + i, so [j, i]
    t = np.clip((q - lo) / span * (n - 1), 0, n - 1 - 1e-9)
    i0, j0 = t[:, 0].astype(int), t[:, 1].astype(int)
    fx, fy = t[:, 0] - i0, t[:, 1] - j0
    return (g[j0, i0] * (1 - fx) * (1 - fy) + g[j0, i0 + 1] * fx * (1 - fy)
            + g[j0 + 1, i0] * (1 - fx) * fy + g[j0 + 1, i0 + 1] * fx * fy)


def dist_to_segments(q, a, b):
    """(nq,) exact distance from each query point to the nearest of the segments (a, b)."""
    ab = b - a  # (ns, 3)
    denom = np.maximum((ab * ab).sum(axis=1), 1e-300)
    d = q[:, None, :] - a[None, :, :]  # (nq, ns, 3)
    t = np.clip((d * ab[None, :, :]).sum(axis=2) / denom[None, :], 0.0, 1.0)
    closest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
    return np.linalg.norm(q[:, None, :] - closest, axis=2).min(axis=1)


def dist_to_triangles(q, t0, t1, t2):
    """(nq,) exact distance from each query point to the nearest of the triangles.

    Point sampling is NOT good enough here: the input complex is frozen at construction
    resolution, so its edges are comparable to delta, and a sampled reference inflates
    the distance by up to half an edge -- measured as avg err 0.30 where the C++ log said
    0.08. So: exact point-triangle distance (Ericson, Real-Time Collision Detection
    5.1.5), vectorized over query x triangle and chunked.
    """
    ab, ac = t1 - t0, t2 - t0  # (nt, 3)
    out = np.empty(len(q))
    for s in range(0, len(q), max(1, 4_000_000 // max(1, len(t0)))):
        p = q[s : s + max(1, 4_000_000 // max(1, len(t0)))]
        ap = p[:, None, :] - t0[None, :, :]  # (np, nt, 3)
        d1 = (ab[None] * ap).sum(2)
        d2 = (ac[None] * ap).sum(2)
        bp = p[:, None, :] - t1[None, :, :]
        d3 = (ab[None] * bp).sum(2)
        d4 = (ac[None] * bp).sum(2)
        cp = p[:, None, :] - t2[None, :, :]
        d5 = (ab[None] * cp).sum(2)
        d6 = (ac[None] * cp).sum(2)

        va = d3 * d6 - d5 * d4
        vb = d5 * d2 - d1 * d6
        vc = d1 * d4 - d3 * d2
        denom = np.maximum(va + vb + vc, 1e-300)

        # Barycentric coordinates of the interior-region projection, then overwrite with
        # each boundary region's clamped result where its condition holds.
        v = vb / denom
        w = vc / denom
        # edge AC (vb region)
        cond = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
        wc = np.clip(np.where(-d6 - d2 != 0, d2 / np.maximum(d2 - d6, 1e-300), 0), 0, 1)
        v = np.where(cond, 0.0, v)
        w = np.where(cond, wc, w)
        # edge BC (va region)
        cond = (va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0)
        wc = np.clip((d4 - d3) / np.maximum((d4 - d3) + (d5 - d6), 1e-300), 0, 1)
        v = np.where(cond, 1.0 - wc, v)
        w = np.where(cond, wc, w)
        # edge AB (vc region)
        cond = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
        vc_ = np.clip(d1 / np.maximum(d1 - d3, 1e-300), 0, 1)
        v = np.where(cond, vc_, v)
        w = np.where(cond, 0.0, w)
        # vertex regions
        cond = (d1 <= 0) & (d2 <= 0)  # A
        v = np.where(cond, 0.0, v)
        w = np.where(cond, 0.0, w)
        cond = (d3 >= 0) & (d4 <= d3)  # B
        v = np.where(cond, 1.0, v)
        w = np.where(cond, 0.0, w)
        cond = (d6 >= 0) & (d5 <= d6)  # C
        v = np.where(cond, 0.0, v)
        w = np.where(cond, 1.0, w)

        closest = t0[None] + v[:, :, None] * ab[None] + w[:, :, None] * ac[None]
        out[s : s + len(p)] = np.linalg.norm(p[:, None, :] - closest, axis=2).min(axis=1)
    return out


FRAME_RE = __import__("re").compile(r"(\d+)(?=\D*$)")


def frame_key(p):
    """Sort debug frames by the counter in their name, not lexically -- debug_9 precedes
    debug_10, which a plain sort gets backwards."""
    m = FRAME_RE.search(p.stem)
    return (int(m.group(1)) if m else -1, p.stem)


def resolve(args):
    """Command line -> (list of mesh paths, config dict or {}, stride)."""
    stride, max_frames, rest = 1, 60, []
    it = iter(args)
    for a in it:
        if a in ("--stride", "-s"):
            stride = max(1, int(next(it)))
        elif a == "--max-frames":
            max_frames = max(1, int(next(it)))
        else:
            rest.append(a)
    paths = [Path(a) for a in rest]

    cfg_path = next((p for p in paths if p.suffix == ".json"), None)
    meshes = [p for p in paths if p.suffix in (".msh", ".vtu")]

    if not meshes:
        d = paths[0] if paths else Path.cwd()
        if not d.is_dir():
            sys.exit("no such file or directory: %s" % d)
        # A SERIES if the directory holds debug frames, otherwise the single result mesh.
        # Frames are what DEBUG_output writes: one .vtu per pass, numbered.
        #
        # A FRAME IS `<name>debug_<N>.vtu` AND NOTHING ELSE. write_vtu() drops companions beside
        # each frame -- `_surf` in 2D, and `_surf`, `_off` and `_edge` in 3D -- carrying the
        # tracked surface, the offset surface and the wire complex on their own. This viewer
        # re-derives all three from the volume mesh, so none of them is a frame.
        #
        # Matching the counter exactly rather than excluding known suffixes: excluding `_surf`
        # alone was right when only 2D wrote frames, and silently tripled the 3D series the day
        # 3D started writing them -- 37 frames read as 111, two thirds of them companions. The
        # `_edge` companion is a wire mesh with no tets, so it has no tagged cells at all and
        # meshio cannot even read it as a volume mesh; the series died on the second frame.
        frames = sorted(
            (f for f in d.glob("*debug_*.vtu") if re.fullmatch(r".*debug_\d+", f.stem)),
            key=frame_key)
        if frames:
            meshes = frames
        else:
            found = sorted(d.glob("*.msh")) or sorted(d.glob("*.vtu"))
            if len(found) != 1:
                sys.exit("expected one .msh/.vtu (or *debug_*.vtu frames) in %s, found %d"
                         % (d, len(found)))
            meshes = found

    if len(meshes) > 1:
        meshes = sorted(meshes, key=frame_key)
        kept = meshes[::stride]
        if len(kept) > max_frames:
            # Say what was dropped rather than silently showing part of the run.
            extra = -(-len(kept) // max_frames)
            print("%d frames after stride %d exceeds --max-frames %d; taking every %d instead"
                  % (len(kept), stride, max_frames, stride * extra))
            kept = meshes[::stride * extra]
        if len(kept) != len(meshes):
            print("series: %d of %d frames (stride %d)" % (len(kept), len(meshes), stride))
        meshes = kept

    if cfg_path is None:
        for c in sorted(meshes[0].parent.glob("*.json")):
            try:
                j = json.load(open(c))
            except Exception:
                continue
            if isinstance(j, dict) and j.get("application") == "topological_offset":
                cfg_path = c
                break
    cfg = {}
    if cfg_path is not None:
        try:
            cfg = json.load(open(cfg_path))
            print("config %s" % cfg_path)
        except Exception as e:
            print("config %s unreadable (%s); using defaults" % (cfg_path, e))
    return meshes, cfg, stride


def load(msh, cfg):
    """Everything main() draws, as plain arrays -- no polyscope."""
    points, dim, groups, envelope = read_any(msh)
    # `offset_tag` is the .vtu frames' label-based band marker and is preferred where
    # present; .msh files name the band by its output tag instead.
    offset_tags = {"offset_tag"} & set(groups)
    if not offset_tags:
        offset_tags = set(cfg.get("offset_output_tags", ["offset"])) & set(groups)
    if not offset_tags:
        offset_tags = {"offset"} & set(groups)

    # THE GROUPS OVERLAP (a cell is written into every group whose tag set contains that
    # tag), and the input complex is not "every non-ambient group": for a selection like
    # "tag_0 & tag_1" it is the INTERSECTION region only, while the sphere-minus-lens
    # parts are ordinary background the offset plows through. Both were measured to
    # matter: union-as-input misfiled the band|sphere interfaces as inner, and read
    # avg err 0.30 where the C++ log said 0.08. So collapse every cell to the C++'s
    # three labels -- offset (2), input complex (1, the selection expression evaluated
    # on the cell's own tag memberships), background (0) -- and derive interfaces from
    # those disjoint classes.
    seen = {}  # sorted vertex tuple -> (row, set of group names)
    for name, cells in groups.items():
        for row in cells:
            k = tuple(sorted(row))
            if k in seen:
                seen[k][1].add(name)
            else:
                seen[k] = (row, {name})
    expr = cfg.get("offset_selection", "!_")
    classed = {"offset": [], "input": [], "ambient": []}
    # The same cells again, deduplicated and in one stable order, so the background mesh can
    # be drawn with per-cell quantities that line up with it: which of the three classes each
    # cell is in, and which tag groups it belongs to. The groups OVERLAP -- a cell is written
    # into every group whose tag set contains that tag -- so tag membership cannot be one
    # scalar and is one boolean layer per tag instead.
    all_rows, all_class, all_names = [], [], []
    for row, names in seen.values():
        if names & offset_tags:
            cls, key = 2, "offset"
        elif eval_selection(expr, names):
            cls, key = 1, "input"
        else:
            cls, key = 0, "ambient"
        classed[key].append(row)
        all_rows.append(row)
        all_class.append(cls)
        all_names.append(names)
    classed = {k: np.asarray(v) if v else np.zeros((0, dim + 1), np.int64)
               for k, v in classed.items()}

    mesh_cells = np.asarray(all_rows) if all_rows else np.zeros((0, dim + 1), np.int64)
    mesh_class = np.asarray(all_class, np.float64)
    mesh_tags = {
        g: np.asarray([1.0 if g in names else 0.0 for names in all_names])
        for g in sorted(groups)
    }

    # REGION BOUNDARIES: facets where the two sides' group-membership sets differ -- the
    # rule label_offset_boundary() classifies edges by. This is what shows the tag-region
    # outlines (and the curve an expression selection lives on) when the selection has no
    # cell region of its own, which is every 2D integration test. Membership sets are
    # mapped to disjoint pseudo-groups so the pairing logic stays count-based and clean;
    # pairs touching the offset class are dropped, the offset layer already draws those.
    memb_groups = {}
    for row, names in seen.values():
        if names & offset_tags:
            key = "__offset"
        else:
            key = "|".join(sorted(names))
        memb_groups.setdefault(key, []).append(row)
    memb_groups = {k: np.asarray(v) for k, v in memb_groups.items()}
    region = [
        f for pair, f in interfaces(points, dim, memb_groups).items()
        if "__offset" not in pair and pair[1] != ""
    ]
    region = np.vstack(region) if region else np.zeros((0, dim), np.int64)

    iface = interfaces(points, dim, classed)

    def pick(*pairs):
        got = [iface[p] for p in pairs if p in iface]
        return np.vstack(got) if got else np.zeros((0, dim), np.int64)

    surf = {
        "input": pick(("ambient", "input"), ("input", "")),
        # The band's outer surface: offset against ambient, plus where the band is
        # clipped at the domain boundary -- both included by the C++ metric.
        "offset": pick(("ambient", "offset"), ("offset", "")),
        "inner": pick(("input", "offset")),
        "region": region,
        "envelope": envelope,
    }

    delta = float(cfg.get("target_distance", -1.0))
    if delta <= 0:
        rel = float(cfg.get("target_distance_rel", 0.01))
        diag = float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))
        delta = rel * diag
        prov = "%g x bbox diagonal %g%s" % (rel, diag, "" if cfg else " (no config: defaults)")
    else:
        prov = "config target_distance"

    # Distance reference: the INNER interface, not the input groups' boundary. For an
    # expression selection (e.g. "tag_0 & tag_1") the complex is the intersection, which
    # the groups alone cannot reproduce -- but the band's inner interface hugs the actual
    # complex by construction, whatever the selection was. Cross-checked on
    # topological_offset_3d: against the union boundary avg err read 0.31 where the C++
    # log says 0.08; against the inner interface it agrees.
    err = None
    ref_facets = surf["inner"] if len(surf["inner"]) else surf["input"]
    if len(surf["offset"]) and len(ref_facets):
        op, oc = compact(points, surf["offset"])
        r = points[ref_facets]
        if dim == 3:
            dist = dist_to_triangles(op, r[:, 0], r[:, 1], r[:, 2])
        else:
            dist = dist_to_segments(op, r[:, 0], r[:, 1])
        err = (op, oc, dist, np.abs(dist - delta) / delta)

    mesh = (mesh_cells, mesh_class, mesh_tags)
    return points, dim, groups, surf, delta, prov, err, mesh


LAYERS = [
    ("background mesh (tags as cell layers)", "background mesh"),
    ("input surface (orange)", "input surface"),
    ("offset surface", "offset surface"),
    ("inner interface (grey)", "inner interface"),
    ("region boundaries (green)", "region boundaries"),
    ("envelope curves (orange)", "envelope curves"),
]


def register_frame(prefix, points, dim, surf, err, mesh):
    """Register one frame's layers, all disabled. Returns {layer label: structure}."""
    mesh_cells, mesh_class, mesh_tags = mesh
    out = {}

    def curve_or_surface(name, facets, color, radius):
        if len(facets) == 0:
            return None
        pts, c = compact(points, facets)
        if dim == 3 and len(facets[0]) == 3:
            s = ps.register_surface_mesh(prefix + name, pts, c, color=color, edge_width=1.0)
        else:
            s = ps.register_curve_network(prefix + name, pts, c, color=color, radius=radius)
        s.set_enabled(False)
        return s

    if len(mesh_cells):
        if dim == 3:
            m = ps.register_volume_mesh(prefix + "background mesh", points, mesh_cells)
        else:
            m = ps.register_surface_mesh(
                prefix + "background mesh", points, mesh_cells,
                color=(0.85, 0.85, 0.87), edge_width=1.0,
            )
        where = "cells" if dim == 3 else "faces"
        m.add_scalar_quantity("class (0 ambient, 1 input, 2 offset)", mesh_class,
                              defined_on=where, cmap="spectral", vminmax=(0.0, 2.0), enabled=True)
        for g, v in mesh_tags.items():
            m.add_scalar_quantity(g, v, defined_on=where, cmap="blues", vminmax=(0.0, 1.0))
        m.set_enabled(False)
        out["background mesh (tags as cell layers)"] = m

    out["input surface (orange)"] = curve_or_surface("input surface", surf["input"], C_INPUT, 0.0022)
    out["offset surface"] = curve_or_surface("offset surface", surf["offset"], C_OFFSET, 0.0022)
    out["inner interface (grey)"] = curve_or_surface("inner interface", surf["inner"], C_INNER, 0.0022)
    out["region boundaries (green)"] = curve_or_surface("region boundaries", surf["region"], C_OTHER, 0.0022)
    out["envelope curves (orange)"] = curve_or_surface("envelope curves", surf["envelope"], C_INPUT, 0.0018)

    if err is not None and len(surf["offset"]):
        _, _, dist, rel = err
        # In 2D the offset surface is a CURVE NETWORK, in 3D a surface mesh; both take vertex
        # scalars, and the error along the offset is the first thing worth looking at.
        m = (ps.get_surface_mesh if dim == 3 else ps.get_curve_network)(prefix + "offset surface")
        m.add_scalar_quantity("|dist - delta| / delta", rel, cmap="reds",
                              vminmax=(0.0, float(rel.max())), enabled=True)
        m.add_scalar_quantity("distance to input", dist, cmap="viridis")
    return {k: v for k, v in out.items() if v is not None}


def load_phi_grid(meshes):
    """The sampled smooth offset potential, written beside the result as `<output>_phi.vtu`.

    THE OFFSET IS A LEVEL SET of a field that exists everywhere, and the result mesh only ever
    samples that field along one curve -- so when the offset lands somewhere unexpected, the mesh
    alone cannot say whether the field is wrong or the optimization failed to reach it. This
    layer is the field itself, drawn as a dense background surface with `phi` as a VERTEX scalar
    so polyscope's isoline mode draws the level set directly.

    Written only when phi_grid_resolution > 0 in the config. Returns None when absent.
    """
    for m in meshes:
        for cand in (m.with_name(m.stem + "_phi.vtu"),
                     m.parent / (m.stem.split("_debug")[0] + "_phi.vtu")):
            if cand.is_file():
                try:
                    grid = meshio.read(str(cand))
                except Exception as e:  # a truncated file should not stop the viewer
                    print("could not read %s: %s" % (cand, e))
                    return None
                cells = np.vstack([b.data for b in grid.cells if b.type == "triangle"])
                return cand, grid.points[:, :2], cells, grid.point_data
    return None


def main():
    meshes, cfg, stride = resolve(sys.argv[1:])
    for m in meshes:
        if not m.is_file():
            sys.exit("missing mesh: %s" % m)
    series = len(meshes) > 1

    # THE RUN'S OWN DISTANCE FIELD, where it wrote one. Everything below prefers it to the
    # distance this viewer can compute for itself, and the difference is not small.
    #
    # The viewer measures the offset against the band's INNER INTERFACE, because the result mesh
    # is all it has -- but the offset band REPLACES the tags on the faces it grows through, so
    # the region the offset was measured from is only partly still tagged in the output. On
    # topological_offset_2d_dragon that is 605 of the input's 1579 cells, and measuring against
    # the survivors reported the worst offset vertex at 92% of delta off target where the run's
    # own number -- against the input as LOADED -- is 5.5%, which an independent check confirmed.
    #
    # <output>_phi.vtu carries `euclidean_distance` sampled from the very BVH the run used, so
    # reading it back is the same measurement rather than a reconstruction of it.
    phi_grid = load_phi_grid(meshes)

    frames = []
    for k, path in enumerate(meshes):
        points, dim, groups, surf, delta, prov, err, mesh = load(path, cfg)
        if phi_grid is not None and len(surf["offset"]):
            op, oc = compact(points, surf["offset"])
            d = sample_regular_grid(
                phi_grid[1], np.asarray(phi_grid[3]["euclidean_distance"]).ravel(), op[:, :2])
            if d is not None:
                err = (op, oc, d, np.abs(d - delta) / delta)
        frames.append((path, points, dim, groups, surf, delta, prov, err, mesh))
        if k == 0 or not series:
            print("mesh   %s  (%s)" % (path, "tets" if dim == 3 else "triangles"))
            print("delta  %g  (%s)" % (delta, prov))
            print("  %-22s %8d cells, %d vertices" % ("background mesh", len(mesh[0]), len(points)))
            for n in sorted(groups):
                print("  group %-16s %8d cells" % (n, len(groups[n])))
            for n in ("input", "offset", "inner", "region", "envelope"):
                print("  %-22s %8d facets" % (n, len(surf[n])))
            if dim == 2:
                print("  (the surface layers are 1D in 2D, so they are polyscope CURVE NETWORKS:"
                      " they are the offset boundary and the input surface, not decoration)")
            if err is not None:
                _, _, dist, rel = err
                print("  offset vertices: dist to input  min %.6g  max %.6g   |err|/delta  avg %.4f  max %.4f  [%s]"
                      % (dist.min(), dist.max(), rel.mean(), rel.max(),
                         "sampled from the run's own distance field"
                         if phi_grid is not None else
                         "measured against the band's inner interface -- see the note in main()"))
            elif not series:
                print("  distance layer omitted: no input-region cells to measure against"
                      " (edge or vertex input complex, or empty surfaces)")
    if series:
        print("series: %d frames, %s .. %s" % (len(frames), meshes[0].name, meshes[-1].name))

    dim = frames[0][2]
    ps.init()
    if dim == 2:
        ps.set_up_dir("y_up")
        ps.set_front_dir("z_front")
        ps.set_view_projection_mode("orthographic")
        ps.set_navigation_style("planar")
    ps.set_ground_plane_mode("none")

    registered = []
    for k, (path, points, d, groups, surf, delta, prov, err, mesh) in enumerate(frames):
        prefix = ("f%04d " % k) if series else ""
        registered.append(register_frame(prefix, points, d, surf, err, mesh))

    # The potential, ONE structure for the whole series: the input complex never moves, so
    # neither does phi, and re-registering it per frame would only cost memory.
    phi_struct = None
    if phi_grid is not None:
        src, pts, cells, pdata = phi_grid
        # meshio hands back (N, 1) for a scalar point field; polyscope wants (N,).
        phi = np.asarray(pdata["phi"]).ravel()
        print("phi grid  %s  (%d samples, %d triangles)" % (src.name, len(pts), len(cells)))
        phi_struct = ps.register_surface_mesh(
            "smooth offset potential", pts, cells, color=(0.85, 0.85, 0.9), edge_width=0.0)
        # THE LEVEL VALUE c is what the isoline has to sit on. The run writes it to its report
        # as `offset_level`; failing that, recover it from the mesh -- the offset boundary IS
        # where phi = c, so phi sampled at the offset-surface vertices is c by definition.
        level = None
        for rep in sorted(src.parent.glob("*_report.json")):
            try:
                level = float(json.load(open(rep)).get("offset_level"))
                break
            except Exception:
                pass
        if level is None:
            off = frames[0][4].get("offset", [])
            if len(off):
                vids = np.unique(np.asarray(off).ravel())
                near = frames[0][1][vids][:, :2]
                idx = [int(np.argmin(((pts - q) ** 2).sum(axis=1))) for q in near[:200]]
                level = float(np.median(phi[idx]))
            else:
                level = float(np.median(phi))

        # Polyscope draws isolines at multiples of the isoline PERIOD, so a period of c puts one
        # exactly on the level set (and the rest at 2c, 3c, ... which read as a contour map).
        q = phi_struct.add_scalar_quantity("phi", phi, cmap="coolwarm", enabled=True,
                                           isolines_enabled=True)
        for setter in ("set_isoline_period", "set_isoline_width"):
            if hasattr(q, setter):
                getattr(q, setter)(level, True)
                break
        phi_struct.add_scalar_quantity(
            "phi residual (length)", np.asarray(pdata["phi_residual_length"]).ravel(),
            cmap="viridis")
        phi_struct.add_scalar_quantity(
            "euclidean distance to input", np.asarray(pdata["euclidean_distance"]).ravel(),
            cmap="viridis")
        phi_struct.set_enabled(False)
        print("  phi at the offset surface (the level c): %.6g" % level)

    # Which LAYERS are on is one choice for the whole series; which FRAME is showing is another.
    # Defaults match the single-mesh viewer: the mesh and the two surfaces, plus the region
    # outlines when there is no input surface to show.
    state = {label: False for label, _ in LAYERS}
    state["background mesh (tags as cell layers)"] = dim == 2
    state["input surface (orange)"] = True
    state["offset surface"] = True
    state["region boundaries (green)"] = len(frames[0][4]["input"]) == 0
    state["envelope curves (orange)"] = True
    state["smooth offset potential"] = False
    state["frame"] = 0
    state["play"] = False
    state["tick"] = 0

    def apply_visibility():
        cur = state["frame"]
        for k, layers in enumerate(registered):
            for label, s in layers.items():
                s.set_enabled(k == cur and state[label])
        if phi_struct is not None:
            phi_struct.set_enabled(state["smooth offset potential"])

    apply_visibility()

    def callback():
        f = frames[state["frame"]]
        psim.TextUnformatted("%s   delta %g" % (f[0].name, f[5]))
        if series:
            psim.TextUnformatted(
                "frame %d / %d   (stride %d)" % (state["frame"] + 1, len(frames), stride))
            changed, v = psim.SliderInt("frame", state["frame"], 0, len(frames) - 1)
            if changed:
                state["frame"] = v
                apply_visibility()
            if psim.Button("prev") and state["frame"] > 0:
                state["frame"] -= 1
                apply_visibility()
            psim.SameLine()
            if psim.Button("next") and state["frame"] + 1 < len(frames):
                state["frame"] += 1
                apply_visibility()
            psim.SameLine()
            _, state["play"] = psim.Checkbox("play", state["play"])
            if state["play"]:
                state["tick"] += 1
                if state["tick"] % 6 == 0:  # ~10 fps at a 60 Hz draw
                    state["frame"] = (state["frame"] + 1) % len(frames)
                    apply_visibility()
        psim.Separator()
        for label, _ in LAYERS:
            if not any(label in layers for layers in registered):
                continue
            changed, state[label] = psim.Checkbox(label, state[label])
            if changed:
                apply_visibility()
        if phi_struct is not None:
            changed, state["smooth offset potential"] = psim.Checkbox(
                "smooth offset potential (phi)", state["smooth offset potential"])
            if changed:
                apply_visibility()

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    main()
