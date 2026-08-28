#!/usr/bin/env python3
"""Polyscope viewer for a topological_offset result: region surfaces + distance error.

    ./visualize_offset.py runs/prism                 # a run directory
    ./visualize_offset.py out.msh                    # mesh alone
    ./visualize_offset.py out.msh config.json        # mesh + the config it ran under
    ./visualize_offset.py runs/prism --lazy          # long series: read frames on demand

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
  bad quality       2D only, off by default: the background triangles whose AMIPS energy is
                    above a threshold, drawn in red with their vertices as red points. The
                    energy is wmtk::AMIPS2D_energy transcribed (2 = equilateral, MAX_ENERGY
                    = 1e50 for inverted or degenerate), so it is the same number stop_energy
                    is compared against -- the threshold starts AT the config's stop_energy
                    and is adjustable on a log10 slider. The background mesh also carries it
                    as a "quality (AMIPS)" cell scalar.

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
import math
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
C_BAD = (0.839, 0.153, 0.157)  # bad-quality highlight; red is reserved for this
C_BAND = (0.45, 0.66, 0.90)  # the band's cells, filled: a lighter form of the front's blue
# One colour per tag for the per-tag boundary layers, in sorted tag order, each named in the
# layer's label. None is the blue of the offset surface, the orange of the input surface, the
# green of the region boundaries, the grey of the inner interface or the red of the bad-quality
# highlight, and no two are the same hue -- the first palette had a cyan beside the blue front
# and two pinks, which read as one thing.
TAG_COLORS = [((0.45, 0.16, 0.62), "purple"), ((0.55, 0.27, 0.07), "brown"),
              ((0.93, 0.79, 0.00), "yellow"), ((0.80, 0.10, 0.45), "magenta"),
              ((0.10, 0.10, 0.10), "black"), ((0.00, 0.45, 0.40), "teal"),
              ((0.60, 0.60, 0.00), "olive"), ((0.95, 0.45, 0.30), "coral")]

# The C++ sentinel for "degenerate, do not trust the number" -- wmtk::TriOptimizerMesh::MAX_ENERGY.
MAX_ENERGY = 1e50


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
    return m.points, ncol - 1, groups, env, point_scalar(m, "sizing")


def point_scalar(m, name):
    """A per-vertex scalar from the file, flattened; None where the writer did not emit it.
    Both write_vtu() debug/phase frames and .msh result files carry `sizing` (m_sizing_scalar
    per vertex); older .msh files written before write_msh_groups() emitted it do not, and for
    those the sizing layer simply is not offered."""
    if name not in getattr(m, "point_data", {}):
        return None
    return np.asarray(m.point_data[name]).reshape(-1)


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
    return m.points, ncol - 1, groups, env, point_scalar(m, "sizing")


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


def amips2d_quality(points, tris):
    """Per-triangle AMIPS energy, TRANSCRIBED FROM THE C++ so the numbers are the same ones.

    This is wmtk::AMIPS2D_energy (src/wmtk/utils/AMIPS2D.cpp) vectorised, wrapped in the guard
    from TriOptimizerMesh::get_quality: a non-finite energy, or one below 2 - 1e-3 (which an
    inverted or degenerate triangle produces), is reported as MAX_ENERGY rather than as a
    small number. Getting that guard wrong would paint inverted triangles as the BEST in the
    mesh, which is the one error that would make this layer actively misleading.

    Matching the C++ is the whole point: it means "bad" here is the same "bad" the optimizer's
    stop_energy is compared against, so a triangle highlighted in the viewer is a triangle the
    run itself is still working on. The energy of an equilateral triangle is 2.

    -> (N,) float array, one entry per row of `tris`.
    """
    p = np.asarray(points, dtype=np.float64)[:, :2]
    t = np.asarray(tris, dtype=np.int64)
    if len(t) == 0:
        return np.zeros(0, dtype=np.float64)
    # The C++ names its temporaries by T index, where T = [x0, y0, x1, y1, x2, y2]; keeping the
    # same letters makes the two readable side by side.
    h1, h3 = p[t[:, 0], 0], p[t[:, 0], 1]   # x0, y0
    h2, h4 = p[t[:, 1], 0], p[t[:, 1], 1]   # x1, y1
    h6, h5 = p[t[:, 2], 0], p[t[:, 2], 1]   # x2, y2
    h7 = 0.666666666666667 * h6
    h8 = 0.666666666666667 * h5
    num = -(
        h1 * (-1.33333333333333 * h1 + 0.666666666666667 * h2 + h7)
        + h2 * (0.666666666666667 * h1 - 1.33333333333333 * h2 + h7)
        + h3 * (-1.33333333333333 * h3 + 0.666666666666667 * h4 + h8)
        + h4 * (0.666666666666667 * h3 - 1.33333333333333 * h4 + h8)
        + h5 * (0.666666666666667 * h3 + 0.666666666666667 * h4 - 1.33333333333333 * h5)
        + h6 * (0.666666666666667 * h1 + 0.666666666666667 * h2 - 1.33333333333333 * h6)
    )
    den = (
        (h1 - h2) * (0.577350269189626 * h3 + 0.577350269189626 * h4 - 1.15470053837925 * h5)
        - (h3 - h4) * (0.577350269189626 * h1 + 0.577350269189626 * h2 - 1.15470053837925 * h6)
    )
    with np.errstate(divide="ignore", invalid="ignore"):
        e = num / den
    bad = ~np.isfinite(e) | (e < 2.0 - 1e-3)
    e[bad] = MAX_ENERGY
    return e


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


# <output>_frames.txt, written beside the frames: one "NNNNN<tab>label" line per frame, the label
# being the compact token r<round><phase><pass>_<operation>. The FILE NAMES are the sequence number
# alone, so ParaView loads them as one time series; what a reader needs to say which round and pass
# a frame belongs to lives here instead.
SEQ_RE = re.compile(r"_(\d{5})$")  # the writer's zero-padded frame counter, at the end of the stem


def read_frame_labels(d):
    """{frame index: label} from <something>_frames.txt in directory `d`; {} if there is none."""
    out = {}
    for man in sorted(Path(d).glob("*_frames.txt")):
        for line in open(man):
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 2 and parts[0].strip().isdigit():
                out[int(parts[0])] = parts[1]
    return out

# The 2D offset's own timeline: <output>_step_<NNNNN>_r<round><A|B><pass>[_<op>] or [_end].
# <op> is the pass the frame follows ("split", "smooth", "collapse-skipped", "B-offset", ...).
# See TopoOffsetTriMesh::write_smoothing_debug_output, which is what names these.
# S is the single-phase mode (alternating_opt false): one loop, so its frames are r<it>S<pass>.
STEP_RE = re.compile(r"step_(\d+)_r(\d+)([ABS])(?:(\d+)(?:_([A-Za-z][A-Za-z-]*))?|_end)$")


TOKEN_RE = re.compile(r"r(\d+)([ABS])(?:(\d+)(?:_([A-Za-z][A-Za-z-]*))?|_end)$")


def step_label(stem, token=None):
    """'r1A3' (a manifest label) or a legacy 'step_00007_r1A3' name -> 'round 1  phase A3'.

    Returns None when neither form matches, so callers can fall back to the file name.
    """
    if token:
        m = TOKEN_RE.search(token)
        if not m:
            return token
        rnd, ph, sub, op = m.groups()
    else:
        m = STEP_RE.search(stem)
        if not m:
            return None
        _, rnd, ph, sub, op = m.groups()
    tail = ("   after " + op) if op else ""
    if int(rnd) == 0:
        return "construction" + tail
    if ph == "S":
        return "iteration %s  pass %s%s" % (rnd, sub if sub else "(end)", tail)
    return "round %s  phase %s%s%s" % (rnd, ph, sub if sub else " (end)", tail)


def frame_key(p):
    """Sort debug frames by the counter in their name, not lexically -- debug_9 precedes
    debug_10, which a plain sort gets backwards.

    A `step_` frame sorts on its GLOBAL counter, which is run order across both phases by
    construction. The trailing-number fallback would sort those on the pass index instead and
    shuffle the phases together, so the step form is matched first.
    """
    m = STEP_RE.search(p.stem)
    if m:
        return (int(m.group(1)), p.stem)
    m = FRAME_RE.search(p.stem)
    return (int(m.group(1)) if m else -1, p.stem)


def resolve(args):
    """Command line -> (list of mesh paths, config dict or {}, stride)."""
    # No frame cap and lazy loading by default for a series (2026-08-26): the process is what
    # is scrubbed through, so every frame at stride 1 is the normal case, and lazy reading is
    # what makes that affordable. --eager restores reading everything up front.
    stride, max_frames, lazy, rest = 1, None, None, []
    tags_on, input_ref = set(), None
    it = iter(args)
    for a in it:
        if a in ("--stride", "-s"):
            stride = max(1, int(next(it)))
        elif a == "--max-frames":
            max_frames = max(1, int(next(it)))
        elif a == "--eager":
            lazy = False
        elif a == "--tags":  # ONLY these per-tag boundary layers on at start (default: all)
            tags_on = set(next(it).split(","))
        elif a == "--input":  # the input mesh, for the fixed per-tag reference curves
            input_ref = Path(next(it))
        elif a == "--lazy":
            lazy = True
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
        # THE PER-PHASE SERIES FIRST: the A/B driver writes `<name>phase_<round><A|B>.vtu`
        # after each phase, which is the timeline of states the phases hand each other --
        # 4 frames for a 2-round run instead of ~150 per-pass ones. The per-pass debug
        # frames remain the fallback for runs made before the phase naming existed (and
        # frame_key orders both: the round is the stem's last number, the A/B tie breaks
        # lexically).
        # THE STEP TIMELINE FIRST: one series covering every phase A and phase B sub-iteration
        # in run order, each frame naming the round and pass it belongs to. That is what the
        # slider scrubs. With DEBUG_output alone it holds just the `_end` frames (two per round);
        # with DEBUG_output_per_pass it holds every pass as well.
        # SEQUENTIALLY NAMED FRAMES + their manifest, which is what the writer produces now.
        labels = read_frame_labels(d)
        frames = []
        if labels:
            # The 5-digit form ONLY: save_vtu also writes <output>_<iteration>.vtu result meshes,
            # and those single-digit numbers would otherwise collide with the frame counter.
            frames = sorted(
                (f for f in d.glob("*.vtu")
                 if SEQ_RE.search(f.stem) and int(SEQ_RE.search(f.stem).group(1)) in labels),
                key=frame_key)
        if not frames:
            frames = sorted(
                (f for f in d.glob("*step_*.vtu") if STEP_RE.search(f.stem)), key=frame_key)
        # The older two-series naming, for runs made before the step naming existed.
        if not frames:
            frames = sorted(
                (f for f in d.glob("*phase_*.vtu") if re.fullmatch(r".*phase_\d+[AB]", f.stem)),
                key=frame_key)
        if not frames:
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
        if max_frames is not None and len(kept) > max_frames:
            # Say what was dropped rather than silently showing part of the run.
            extra = -(-len(kept) // max_frames)
            print("%d frames after stride %d exceeds --max-frames %d; taking every %d instead"
                  % (len(kept), stride, max_frames, stride * extra))
            kept = meshes[::stride * extra]
            stride = stride * extra  # the panel reports the EFFECTIVE stride, not the requested one
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
    if lazy is None:
        lazy = len(meshes) > 1
    # The reference input defaults to the config's own "input", relative to the config file.
    if input_ref is None and cfg.get("input"):
        cand = Path(cfg["input"])
        if not cand.is_absolute() and cfg_path is not None:
            cand = (cfg_path.parent / cand).resolve()
        input_ref = cand
    return meshes, cfg, stride, lazy, tags_on, input_ref


def load(msh, cfg):
    """Everything main() draws, as plain arrays -- no polyscope."""
    points, dim, groups, envelope, sizing = read_any(msh)
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
    # PER-TAG BOUNDARIES, one curve per tag, from THIS frame's tags: the boundary of the set of
    # cells carrying the tag, whatever else they carry. The classes above come from
    # offset_selection, which is the right view of the offset and a misleading one of the
    # input: on a multi-tag model (the dragon fixture) "input surface" is the outline of
    # everything the selection matches -- every tag, when no config is found -- and where the
    # band overwrote tags it is cropped at the band and moves as cells beside it collapse.
    # These curves say, per tag, what that tag's cells look like right now.
    # Not for the band (any of its names) and not for ambient, whose "boundary" is the domain
    # box plus the outline of everything tagged -- the layer that read as the input surface
    # showing the box.
    skip = offset_tags | {"offset_tag", "offset", "ambient"} | set(cfg.get("offset_output_tags", []))
    for g in sorted(groups):
        if g in skip:
            continue
        without = [row for row, names in seen.values() if g not in names]
        parts = {"in": groups[g]}
        if without:
            parts["out"] = np.asarray(without)
        tb = interfaces(points, dim, parts)
        got = [tb[p] for p in (("in", "out"), ("in", "")) if p in tb]
        surf["tag:" + g] = np.vstack(got) if got else np.zeros((0, dim), np.int64)

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
    return points, dim, groups, surf, delta, prov, err, mesh, sizing


LAYERS = [
    ("background mesh (tags as cell layers)", "background mesh"),
    ("offset band (filled, pale blue)", "offset band"),
    ("input surface (orange)", "input surface"),
    ("offset surface", "offset surface"),
    ("inner interface (grey)", "inner interface"),
    ("region boundaries (green)", "region boundaries"),
    ("envelope curves (orange)", "envelope curves"),
]


def register_frame(prefix, points, dim, surf, err, mesh, sizing=None):
    """Register one frame's layers, all disabled.

    Returns ({layer label: structure}, extras) where extras holds what the sizing toggle needs
    to re-add quantities with the desired enabled state: `off` = (offset-surface structure,
    its compacted sizing rows, (err values, err max) or None) and `bg` = (background structure,
    sizing, class values, defined_on). Re-adding rather than handles, because this polyscope's
    add_scalar_quantity returns None -- and re-adding under the same name is the API's setter.
    The displaced quantity must be restored on untoggle: polyscope keeps one active scalar per
    structure, so enabling one silently disables the other and unchecking would otherwise leave
    the structure colorless.
    """
    mesh_cells, mesh_class, mesh_tags = mesh
    out = {}
    extras = {}

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
        # OFF by default (2026-08-26): with it on, every cell that is neither source nor band --
        # on a multi-tag model, other regions' cells -- is painted the colormap's low end, a
        # magenta that drowned the curve layers. The mesh is flat grey unless it is switched on.
        m.add_scalar_quantity("class (0 ambient, 1 input, 2 offset)", mesh_class,
                              defined_on=where, cmap="spectral", vminmax=(0.0, 2.0), enabled=False)
        for g, v in mesh_tags.items():
            m.add_scalar_quantity(g, v, defined_on=where, cmap="blues", vminmax=(0.0, 1.0))
        if sizing is not None:
            # The sizing scalar is RELATIVE (a fraction of the target edge length, floored by
            # min_sizing_scalar), so a fixed 0..1 range keeps the colors comparable across
            # frames -- which is the point of showing it on a timeline.
            m.add_scalar_quantity("sizing", sizing, cmap="viridis", vminmax=(0.0, 1.0))
            extras["bg"] = (m, sizing, mesh_class, where)
        if dim == 2:
            # AMIPS per triangle, on the same scale stop_energy is stated in. Capped for the
            # COLOUR RAMP only -- the values the highlight thresholds against are the uncapped
            # ones -- because a single degenerate triangle at MAX_ENERGY would otherwise flatten
            # the entire ramp to one colour.
            q = amips2d_quality(points, mesh_cells)
            finite = q[q < MAX_ENERGY]
            top = float(finite.max()) if len(finite) else 2.0
            m.add_scalar_quantity("quality (AMIPS, 2 = equilateral)", np.minimum(q, top),
                                  defined_on=where, cmap="reds", vminmax=(2.0, max(top, 2.0 + 1e-9)))
            extras["quality"] = (points, mesh_cells, q)
        m.set_enabled(False)
        out["background mesh (tags as cell layers)"] = m
        # THE BAND'S CELLS, FILLED. Everything else stays grey, so the region the offset owns
        # right now reads at a glance; the front (blue curve) is its edge.
        band_cells = mesh_cells[mesh_class == 2.0]
        if len(band_cells):
            if dim == 3:
                b = ps.register_volume_mesh(prefix + "offset band", points, band_cells, color=C_BAND)
            else:
                # Lifted a hair toward the camera: in the same plane as the background mesh it
                # z-fights with it and the grey wins. Relative to the scene size, like the tube
                # radii; still under the curve tubes.
                lift = np.zeros_like(points)
                lift[:, 2] = 5e-4 * float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))
                b = ps.register_surface_mesh(prefix + "offset band", points + lift, band_cells,
                                             color=C_BAND, edge_width=1.0)
            b.set_enabled(False)
            out["offset band (filled, pale blue)"] = b

    out["input surface (orange)"] = curve_or_surface("input surface", surf["input"], C_INPUT, 0.0022)
    out["offset surface"] = curve_or_surface("offset surface", surf["offset"], C_OFFSET, 0.0022)
    out["inner interface (grey)"] = curve_or_surface("inner interface", surf["inner"], C_INNER, 0.0022)
    out["region boundaries (green)"] = curve_or_surface("region boundaries", surf["region"], C_OTHER, 0.0022)
    out["envelope curves (orange)"] = curve_or_surface("envelope curves", surf["envelope"], C_INPUT, 0.0018)
    for i, key in enumerate(sorted(k for k in surf if k.startswith("tag:"))):
        g = key[len("tag:"):]
        color, cname = TAG_COLORS[i % len(TAG_COLORS)]
        out["tag boundary: %s (%s)" % (g, cname)] = curve_or_surface(
            "tag boundary " + g, surf[key], color, 0.0007)

    if err is not None and len(surf["offset"]):
        _, _, dist, rel = err
        # In 2D the offset surface is a CURVE NETWORK, in 3D a surface mesh; both take vertex
        # scalars, and the error along the offset is the first thing worth looking at.
        m = (ps.get_surface_mesh if dim == 3 else ps.get_curve_network)(prefix + "offset surface")
        # OFF by default (2026-08-26): "reds" is white at zero error, so a well-placed front drew
        # as a white/grey tube. Plain blue says where the front is; switch this on to see how far
        # from delta it is.
        m.add_scalar_quantity("|dist - delta| / delta", rel, cmap="reds",
                              vminmax=(0.0, float(rel.max())), enabled=False)
        m.add_scalar_quantity("distance to input", dist, cmap="viridis")
    if sizing is not None and out.get("offset surface") is not None:
        # The same compaction the surface itself was registered with: its vertices are
        # points[unique(facets)] in unique() order, so the sizing rows follow that order.
        m = (ps.get_surface_mesh if dim == 3 else ps.get_curve_network)(prefix + "offset surface")
        s_off = sizing[np.unique(surf["offset"])]
        m.add_scalar_quantity("sizing", s_off, cmap="viridis", vminmax=(0.0, 1.0))
        rel_info = (err[3], float(err[3].max())) if err is not None else None
        extras["off"] = (m, s_off, rel_info)
    return {k: v for k, v in out.items() if v is not None}, extras


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
    meshes, cfg, stride, lazy, tags_on, input_ref = resolve(sys.argv[1:])
    frame_labels = read_frame_labels(meshes[0].parent) if meshes else {}
    for m in meshes:
        if not m.is_file():
            sys.exit("missing mesh: %s" % m)
    series = len(meshes) > 1
    lazy = lazy and series
    n_frames = len(meshes)

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

    def load_frame(path):
        points, dim, groups, surf, delta, prov, err, mesh, sizing = load(path, cfg)
        if phi_grid is not None and len(surf["offset"]):
            op, oc = compact(points, surf["offset"])
            d = sample_regular_grid(
                phi_grid[1], np.asarray(phi_grid[3]["euclidean_distance"]).ravel(), op[:, :2])
            if d is not None:
                err = (op, oc, d, np.abs(d - delta) / delta)
        return (path, points, dim, groups, surf, delta, prov, err, mesh, sizing)

    # --lazy: a series is read ONE FRAME AT A TIME, when the slider lands on it, instead of
    # every frame up front. Both halves of the eager path are the slow part -- load() parses
    # the file, extracts the interfaces and measures the distance field, and then every frame
    # is a registered polyscope structure -- so on a run of several hundred passes the eager
    # window takes minutes to open and scrubbing drags. Lazy costs a short pause the first
    # time a frame is visited; the last few visited stay cached.
    cache = {}

    def frame_at(k):
        if k not in cache:
            if len(cache) >= 8:
                del cache[next(iter(cache))]
            cache[k] = load_frame(meshes[k])
        return cache[k]

    frames = []
    for k, path in enumerate(meshes[:1] if lazy else meshes):
        fr = frame_at(k) if lazy else load_frame(path)
        frames.append(fr)
        path, points, dim, groups, surf, delta, prov, err, mesh, sizing = fr
        if k == 0 or not series:
            print("mesh   %s  (%s)" % (path, "tets" if dim == 3 else "triangles"))
            print("delta  %g  (%s)" % (delta, prov))
            print("  'input surface' = boundary of the cells offset_selection %r matches%s"
                  % (cfg.get("offset_selection", "!_"),
                     "" if cfg else "  -- NO CONFIG FOUND, so every tag counts; put the run's "
                                    "json next to the frames"))
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
        print("series: %d frames, %s .. %s%s" % (n_frames, meshes[0].name, meshes[-1].name,
                                                  "  (lazy: read on demand)" if lazy else ""))

    dim = frames[0][2]
    ps.init()
    if dim == 2:
        ps.set_up_dir("y_up")
        ps.set_front_dir("z_front")
        ps.set_view_projection_mode("orthographic")
        ps.set_navigation_style("planar")
    ps.set_ground_plane_mode("none")

    registered = []
    extra_qs = [] # per frame: quantity handles the sizing toggle flips, see register_frame
    for k, (path, points, d, groups, surf, delta, prov, err, mesh, sizing) in enumerate(frames):
        prefix = ("f%04d " % k) if series and not lazy else ""
        layers, extras = register_frame(prefix, points, d, surf, err, mesh, sizing)
        registered.append(layers)
        extra_qs.append(extras)
    has_sizing = any(("off" in ex or "bg" in ex) for ex in extra_qs)

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

    # THE INPUT GEOMETRY, per tag, as a fixed reference: the boundary of each physical group of
    # the input mesh (--input, or the config's "input"), registered once. The frames' own tag
    # boundaries move -- the band overwrites tags and Phase A collapses cells -- so "has the
    # dragon changed?" needs the input drawn beside it, and this is the input as the run loaded it.
    ref_structs = []
    if input_ref is not None and input_ref.is_file():
        rp, rdim, rgroups, _, _ = read_groups(input_ref)
        rows = {}
        for g, cells in rgroups.items():
            for r in cells:
                rows.setdefault(tuple(sorted(r)), [r, set()])[1].add(g)
        for i, g in enumerate(sorted(g for g in rgroups if g != "ambient")):
            without = [r for r, names in rows.values() if g not in names]
            parts = {"in": rgroups[g]}
            if without:
                parts["out"] = np.asarray(without)
            tb = interfaces(rp, rdim, parts)
            got = [tb[p] for p in (("in", "out"), ("in", "")) if p in tb]
            if not got:
                continue
            pts_, c_ = compact(rp, np.vstack(got))
            # Same hue as the frame's own layer for that tag, THIN: thick = this frame, thin = input.
            color = TAG_COLORS[i % len(TAG_COLORS)][0]
            name = "input %s (reference)" % g
            if rdim == 3:
                s = ps.register_surface_mesh(name, pts_, c_, color=color, edge_width=1.0)
            else:
                s = ps.register_curve_network(name, pts_, c_, color=color, radius=0.0007)
            s.set_enabled(False)
            ref_structs.append(s)
        print("input geometry (reference): %s, %d tag boundaries" % (input_ref, len(ref_structs)))
    elif input_ref is not None:
        print("input geometry (reference): %s not found" % input_ref)

    # Which LAYERS are on is one choice for the whole series; which FRAME is showing is another.
    # Defaults match the single-mesh viewer: the mesh and the two surfaces, plus the region
    # outlines when there is no input surface to show.
    state = {label: False for label, _ in LAYERS}
    state["background mesh (tags as cell layers)"] = dim == 2
    state["input surface (orange)"] = True
    state["offset surface"] = True
    state["offset band (filled, pale blue)"] = True
    state["region boundaries (green)"] = len(frames[0][4]["input"]) == 0
    state["envelope curves (orange)"] = True
    state["smooth offset potential"] = False
    state["sizing field"] = False
    state["frame"] = 0
    state["play"] = False
    state["tick"] = 0
    state["input geometry (reference)"] = bool(ref_structs)
    # THE THRESHOLD DEFAULTS TO THE RUN'S OWN BAR. stop_energy is what Phase A is trying to get
    # every triangle under, so "bad" out of the box means "the run is not done with this one",
    # not an arbitrary cutoff. The offset spec's default is 100; a config that set it wins.
    state["bad quality (red)"] = False
    state["bad threshold"] = float(cfg.get("stop_energy", 100.0))

    has_quality = any("quality" in ex for ex in extra_qs)

    # Handles to the highlight structures, kept rather than looked up by name: ps.has_*() is not
    # in every polyscope release, and holding the handle needs no such query.
    bad_structs = {}

    def rebuild_bad():
        """Re-register the highlight for the CURRENT frame and threshold.

        Rebuilt rather than toggled, because the SET itself depends on the threshold slider --
        there is no persistent structure to just show and hide. Registering under the same name
        replaces the previous one, which is this polyscope's setter, the same idiom the sizing
        toggle uses for scalars.
        """
        if not has_quality:
            return
        cur = 0 if lazy else state["frame"]
        ex = extra_qs[cur] if cur < len(extra_qs) else {}
        sel = None
        if state["bad quality (red)"] and "quality" in ex:
            pts, cells, q = ex["quality"]
            sel = q > state["bad threshold"]
        if sel is None or not sel.any():
            for h in bad_structs.values():
                h.set_enabled(False)
            return
        bad_cells = np.asarray(cells)[sel]
        # Drawn on the FULL point array rather than a compacted one, so the highlight sits
        # exactly on top of the background triangles it came from instead of being a separate
        # object that has to be lined up by eye. Polyscope tolerates unreferenced vertices.
        sm = ps.register_surface_mesh("bad quality tris", pts, bad_cells,
                                      color=C_BAD, edge_width=1.0)
        sm.set_enabled(True)
        bad_structs["tris"] = sm
        pc = ps.register_point_cloud("bad quality verts", pts[np.unique(bad_cells)])
        pc.set_color(C_BAD)
        pc.set_enabled(True)
        bad_structs["verts"] = pc

    def apply_visibility():
        cur = 0 if lazy else state["frame"]
        for k, layers in enumerate(registered):
            for label, s in layers.items():
                s.set_enabled(k == cur and state.get(label, False))
        rebuild_bad()  # the highlight follows the frame, so it is rebuilt with it
        if phi_struct is not None:
            phi_struct.set_enabled(state["smooth offset potential"])
        for s in ref_structs:
            s.set_enabled(state["input geometry (reference)"])
        # The sizing toggle swaps which scalar the visible structures are colored by --
        # set on EVERY frame, not just the current one, so scrubbing keeps the choice.
        # Re-adding under the same name is this polyscope's setter; see register_frame.
        on = state["sizing field"]
        for ex in extra_qs:
            if "off" in ex:
                m, s_off, rel_info = ex["off"]
                m.add_scalar_quantity("sizing", s_off, cmap="viridis", vminmax=(0.0, 1.0),
                                      enabled=on)
                if not on and rel_info is not None:
                    rel, rmax = rel_info
                    m.add_scalar_quantity("|dist - delta| / delta", rel, cmap="reds",
                                          vminmax=(0.0, rmax), enabled=False)
            if "bg" in ex:
                m, s, cls, where = ex["bg"]
                m.add_scalar_quantity("sizing", s, cmap="viridis", vminmax=(0.0, 1.0),
                                      enabled=on)
                if not on:
                    m.add_scalar_quantity("class (0 ambient, 1 input, 2 offset)", cls,
                                          defined_on=where, cmap="spectral",
                                          vminmax=(0.0, 2.0), enabled=False)

    apply_visibility()

    def show_frame(v):
        state["frame"] = v
        if lazy:
            # Swap the one registered frame: drop the old structures by handle (a layer the
            # new frame lacks would otherwise linger), then register the new frame under the
            # same names.
            for s in registered[0].values():
                try:
                    s.remove()
                except Exception:
                    pass
            fr = frame_at(v)
            registered[0], extra_qs[0] = register_frame("", fr[1], fr[2], fr[4], fr[7], fr[8], fr[9])
        apply_visibility()

    def callback():
        f = frame_at(state["frame"]) if lazy else frames[state["frame"]]
        idx_m = SEQ_RE.search(f[0].stem)
        tok = frame_labels.get(int(idx_m.group(1))) if idx_m else None
        lbl = step_label(f[0].stem, tok)
        if lbl:
            # THE SUB-ITERATION, not the file name: which A/B round and which pass inside which
            # phase is the thing being scrubbed through, so it goes first and largest.
            psim.TextUnformatted(lbl)
            psim.TextUnformatted("%s   delta %g" % (f[0].name, f[5]))
        else:
            psim.TextUnformatted("%s   delta %g" % (f[0].name, f[5]))
        if series:
            psim.TextUnformatted(
                "frame %d / %d   (stride %d)" % (state["frame"] + 1, n_frames, stride))
            changed, v = psim.SliderInt("frame", state["frame"], 0, n_frames - 1)
            if changed:
                show_frame(v)
            if psim.Button("prev") and state["frame"] > 0:
                show_frame(state["frame"] - 1)
            psim.SameLine()
            if psim.Button("next") and state["frame"] + 1 < n_frames:
                show_frame(state["frame"] + 1)
            psim.SameLine()
            _, state["play"] = psim.Checkbox("play", state["play"])
            if state["play"]:
                state["tick"] += 1
                if state["tick"] % 6 == 0:  # ~10 fps at a 60 Hz draw
                    show_frame((state["frame"] + 1) % n_frames)
        psim.Separator()
        for label, _ in LAYERS:
            if not any(label in layers for layers in registered):
                continue
            changed, state[label] = psim.Checkbox(label, state[label])
            if changed:
                apply_visibility()
        tag_labels = sorted({l for layers in registered for l in layers
                             if l.startswith("tag boundary: ")})
        if tag_labels:
            psim.Separator()
            psim.TextUnformatted("boundary of each tag's cells, from this frame")
            for label in tag_labels:
                tag_name = label[len("tag boundary: "):].rsplit(" (", 1)[0]
                state.setdefault(label, (tag_name in tags_on) if tags_on else True)
                changed, state[label] = psim.Checkbox(label, state[label])
                if changed:
                    apply_visibility()
        if ref_structs:
            changed, state["input geometry (reference)"] = psim.Checkbox(
                "input geometry per tag (reference, thin)", state["input geometry (reference)"])
            if changed:
                apply_visibility()
        if phi_struct is not None:
            changed, state["smooth offset potential"] = psim.Checkbox(
                "smooth offset potential (phi)", state["smooth offset potential"])
            if changed:
                apply_visibility()
        if has_sizing:
            changed, state["sizing field"] = psim.Checkbox(
                "sizing field (viridis, 0-1)", state["sizing field"])
            if changed:
                apply_visibility()
        if has_quality:
            changed, state["bad quality (red)"] = psim.Checkbox(
                "bad quality tris + their vertices (red)", state["bad quality (red)"])
            if changed:
                rebuild_bad()
            if state["bad quality (red)"]:
                # Log scale: AMIPS runs from 2 for an equilateral triangle to MAX_ENERGY (1e50)
                # for an inverted one, so a linear slider would spend its whole travel in the
                # first pixel. Powers of ten are also how stop_energy is actually chosen.
                lo, hi = 0.301029995663981, 6.0  # log10(2) .. log10(1e6)
                cur = max(lo, min(hi, math.log10(max(state["bad threshold"], 2.0))))
                ch2, v = psim.SliderFloat("AMIPS threshold (log10)", cur, lo, hi)
                if ch2:
                    state["bad threshold"] = 10.0 ** v
                    rebuild_bad()
                ex = extra_qs[state["frame"]] if state["frame"] < len(extra_qs) else {}
                if "quality" in ex:
                    q = ex["quality"][2]
                    n_bad = int((q > state["bad threshold"]).sum())
                    n_deg = int((q >= MAX_ENERGY).sum())
                    psim.TextUnformatted(
                        "  threshold %.4g   ->  %d / %d tris bad (%d degenerate/inverted)"
                        % (state["bad threshold"], n_bad, len(q), n_deg))

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    main()
