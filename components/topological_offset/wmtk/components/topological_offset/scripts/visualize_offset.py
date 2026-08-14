#!/usr/bin/env python3
"""Polyscope viewer for a topological_offset result: region surfaces + distance error.

    ./visualize_offset.py runs/prism                 # a run directory
    ./visualize_offset.py out.msh                    # mesh alone
    ./visualize_offset.py out.msh config.json        # mesh + the config it ran under

Needs polyscope, meshio and numpy (same venv as visualize_triwild.py; see the triwild
scripts README).

The output .msh holds one cell block per physical group -- `ambient`, one group per tag,
and the offset output tag(s). Nothing in the file marks the surfaces the optimization is
about, so they are derived here the way the C++ derives them from the labels: an
interface face (3D) or edge (2D) is one whose two incident cells lie in different
groups, and

  input surface     boundary of the non-ambient, non-offset groups -- the complex the
                    offset is measured against (orange)
  offset surface    offset group against ambient: the band's OUTER surface, the one
                    supposed to sit at target_distance (blue)
  inner interface   offset group against the input groups: hugs the complex at distance
                    ~0 by construction, drawn dim and off by default, so it cannot be
                    mistaken for the offset
  <other tags>      any remaining group boundary, off by default

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
    """(points, dim, {group name: cell array}) from a topological_offset .msh."""
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
    return m.points, ncol - 1, groups


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


def resolve(args):
    """Command line -> (msh path, config dict or {})."""
    paths = [Path(a) for a in args]
    msh = next((p for p in paths if p.suffix == ".msh"), None)
    cfg_path = next((p for p in paths if p.suffix == ".json"), None)
    if msh is None:
        d = paths[0] if paths else Path.cwd()
        if not d.is_dir():
            sys.exit("no such file or directory: %s" % d)
        found = sorted(d.glob("*.msh"))
        if len(found) != 1:
            sys.exit("expected exactly one .msh in %s, found %d" % (d, len(found)))
        msh = found[0]
    if cfg_path is None:
        # Any json beside the mesh that declares the right application.
        for c in sorted(msh.parent.glob("*.json")):
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
    return msh, cfg


def load(msh, cfg):
    """Everything main() draws, as plain arrays -- no polyscope."""
    points, dim, groups = read_groups(msh)
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
    for row, names in seen.values():
        if names & offset_tags:
            classed["offset"].append(row)
        elif eval_selection(expr, names):
            classed["input"].append(row)
        else:
            classed["ambient"].append(row)
    classed = {k: np.asarray(v) if v else np.zeros((0, dim + 1), np.int64)
               for k, v in classed.items()}

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

    return points, dim, groups, surf, delta, prov, err


def main():
    msh, cfg = resolve(sys.argv[1:])
    if not msh.is_file():
        sys.exit("missing mesh: %s" % msh)
    points, dim, groups, surf, delta, prov, err = load(msh, cfg)

    print("mesh   %s  (%s)" % (msh, "tets" if dim == 3 else "triangles"))
    print("delta  %g  (%s)" % (delta, prov))
    for n in sorted(groups):
        print("  group %-16s %8d cells" % (n, len(groups[n])))
    for n in ("input", "offset", "inner"):
        print("  %-22s %8d facets" % (n + " surface", len(surf[n])))
    if err is not None:
        _, _, dist, rel = err
        print("  offset vertices: dist to input  min %.6g  max %.6g   |err|/delta  avg %.4f  max %.4f"
              % (dist.min(), dist.max(), rel.mean(), rel.max()))
        print("  (reference = the band's inner interface, sampled at offset vertices:"
              " a cross-check against the log, not the metric itself)")
    else:
        print("  distance layer omitted: no input-region cells to measure against"
              " (edge or vertex input complex, or empty surfaces)")

    ps.init()
    if dim == 2:
        # Flat data: same setup and rationale as visualize_triwild.py.
        ps.set_up_dir("y_up")
        ps.set_front_dir("z_front")
        ps.set_view_projection_mode("orthographic")
        ps.set_navigation_style("planar")
    ps.set_ground_plane_mode("none")

    def register(name, facets, color, enabled):
        if len(facets) == 0:
            return None
        p, c = compact(points, facets)
        if dim == 3:
            s = ps.register_surface_mesh(name, p, c, color=color, edge_width=1.0)
        else:
            s = ps.register_curve_network(name, p, c, color=color, radius=0.0022)
        s.set_enabled(enabled)
        return s

    layers = [
        ("input surface (orange)", register("input surface", surf["input"], C_INPUT, True)),
        ("offset surface (blue)", register("offset surface", surf["offset"], C_OFFSET, True)),
        ("inner interface (grey)", register("inner interface", surf["inner"], C_INNER, False)),
    ]
    if err is not None and dim == 3:
        op, oc, dist, rel = err
        s = ps.get_surface_mesh("offset surface")
        s.add_scalar_quantity("distance to input", dist, cmap="viridis")
        s.add_scalar_quantity("|dist - delta| / delta", rel, cmap="reds", enabled=True)

    state = {label: (s is not None and s.is_enabled()) for label, s in layers}

    def callback():
        psim.TextUnformatted("%s   delta %g" % (msh.name, delta))
        psim.Separator()
        for label, s in layers:
            if s is None:
                continue
            changed, state[label] = psim.Checkbox(label, state[label])
            if changed:
                s.set_enabled(state[label])

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    main()
