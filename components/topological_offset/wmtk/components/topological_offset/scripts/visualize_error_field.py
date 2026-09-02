#!/usr/bin/env python3
"""Pixel image of the offset's quadratic error E = (Phi - c)^2 over a 2D input mesh.

    ./visualize_error_field.py two_circles.msh                     # defaults
    ./visualize_error_field.py two_circles.msh config.json         # delta + selection from it
    ./visualize_error_field.py two_circles.msh --cap 0.05 --res 800 --save out.png

Needs matplotlib, meshio and numpy -- the same environment visualize_offset.py wants:

    /Volumes/Seagate_Drive/offsets_optimization_testing/.venv/bin/python

WHAT IT DOES. Evaluates the potential on a regular grid and draws one pixel per sample,
coloured by value. Nothing else -- no contours, no overlays unless asked for.

  --cap V     values above V are clamped to V before colouring, so the colour range is
              [min, V]. Phi is a BARRIER: it diverges ON the input complex, so a handful of
              pixels next to it are orders of magnitude above everything else and take the
              whole range uncapped. Default is c^2, which is what E equals where Phi = 0.
  --plot      `error` (default) draws E = (Phi - c)^2; `phi` draws Phi itself.
  --res       pixels on the long axis.

WHICH Phi. Both fields, selected with --field (default: the config's offset_field, else
`smooth`, which is the component's own default).

  euclidean   exact distance to the input complex. Level c = target_distance.
  smooth      the ipc-toolkit high_order_contact (OGC) potential -- the SAME object the run
              uses, reached through the wmtk_offset_field extension module rather than
              reimplemented, because a Python approximation of it would be a different field
              wearing the same name. Build it with:
                  cmake --build build --target wmtk_offset_field
              and this script finds it in <repo>/build/bin automatically. It is IDENTICALLY
              ZERO past dhat = offset_dhat_factor x delta; the exact distance to the complex
              (cheap in numpy) is used to skip calling it on samples out there, which is not an
              approximation and is most of the grid.

WHICH SEGMENTS. The input complex is derived from the mesh the way the C++ derives it: an edge
whose two incident triangles have different group membership under the selection. With a config,
`offset_selection` is used; without one, the default is every non-ambient group against the rest.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np


def load_field_module():
    """The wmtk_offset_field extension, from the repo's build/bin. -> module or None.

    Located relative to this file rather than requiring PYTHONPATH, since the build tree is
    always at a fixed depth above scripts/.
    """
    here = Path(__file__).resolve()
    for up in here.parents:
        cand = up / "build" / "bin"
        if cand.is_dir():
            sys.path.insert(0, str(cand))
            break
    try:
        import wmtk_offset_field
        return wmtk_offset_field
    except ImportError:
        return None


# ---------------------------------------------------------------------------------------------
# input


def eval_selection(expr, names):
    """The offset_selection boolean expression, on one cell's group memberships.

    Same language the C++ parses: tag names, & | !, parentheses; 'ambient' (spelled '_' in the
    spec) means "in no tag group". Names are substituted with their membership truth value, so
    eval only ever sees booleans and the three operators.
    """
    import re

    def sub(m):
        tok = m.group(0)
        if tok in ("ambient", "_"):
            return str(not (names - {"ambient"}))
        return str(tok in names)

    py = re.sub(r"[A-Za-z0-9_]+", sub, expr)  # tag names may be purely numeric
    py = py.replace("!", " not ").replace("&", " and ").replace("|", " or ")
    if not re.fullmatch(r"[ ()TrueFalsandnot]*", py):
        raise ValueError("cannot evaluate offset_selection %r" % expr)
    return bool(eval(py))


def read_mesh(path):
    """-> (points (n,2), triangles (m,3), {group name: bool mask over triangles})."""
    import meshio

    m = meshio.read(str(path))
    pts = m.points[:, :2]
    blocks = [(i, c) for i, c in enumerate(m.cells) if c.type == "triangle"]
    if not blocks:
        sys.exit("%s has no triangles -- this script is 2D only" % path)
    tris = np.vstack([c.data for _, c in blocks])

    # gmsh physical ids -> names, restricted to the 2D entities
    id_to_name = {int(v[0]): k for k, v in m.field_data.items() if int(v[1]) == 2}
    key = "gmsh:physical"
    if key not in m.cell_data:
        return pts, tris, {}
    phys = np.concatenate([np.asarray(m.cell_data[key][i]).ravel() for i, _ in blocks])
    groups = {name: (phys == gid) for gid, name in id_to_name.items()}
    return pts, tris, groups


def selection_mask(tris, groups, expr):
    """(m,) bool: which triangles the offset_selection picks out. This is the region being
    offset -- the offset grows OUT of it, so it is also the region there is nothing to plot in."""
    names = sorted(groups)
    sel = np.zeros(len(tris), dtype=bool)
    for i in range(len(tris)):
        mem = {n for n in names if groups[n][i]}
        sel[i] = eval_selection(expr, mem)
    if not sel.any():
        sys.exit("selection %r matched no triangles (groups: %s)" % (expr, ", ".join(names)))
    return sel


def input_segments(pts, tris, sel, expr):
    """The input complex: edges whose two incident triangles differ under the selection.

    Returns (a, b) as two (ns, 2) arrays of segment endpoints. A boundary edge of the domain
    counts only if its one triangle is selected -- the outside is 'not selected' by definition.
    """
    side = {}
    for t, s in zip(tris, sel):
        for u, v in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            k = (u, v) if u < v else (v, u)
            side.setdefault(k, []).append(bool(s))
    edges = [k for k, vs in side.items()
             if (len(vs) == 2 and vs[0] != vs[1]) or (len(vs) == 1 and vs[0])]
    if not edges:
        sys.exit("selection %r has no boundary -- it is the whole mesh or none of it" % expr)
    e = np.array(edges, dtype=np.int64)
    return pts[e[:, 0]], pts[e[:, 1]], e


def in_selected_region(q, pts, tris, sel):
    """(nq,) bool: which query points land inside a SELECTED triangle.

    Exact point location through matplotlib's trapezoid-map trifinder -- O(log m) per query and
    vectorised -- rather than a point-in-triangle sweep, which at 10^6 pixels against 10^4
    triangles would be 10^10 tests. Points outside the mesh entirely come back as -1.
    """
    from matplotlib.tri import Triangulation

    finder = Triangulation(pts[:, 0], pts[:, 1], tris).get_trifinder()
    idx = finder(np.ascontiguousarray(q[:, 0]), np.ascontiguousarray(q[:, 1]))
    return (idx >= 0) & sel[np.maximum(idx, 0)]


# ---------------------------------------------------------------------------------------------
# the field


def dist_to_segments(q, a, b, chunk=20000):
    """(nq,) exact distance from each query point to the nearest segment.

    Chunked over the query points: the intermediate is (chunk, ns, 2), and at 600^2 samples
    against a few hundred segments the unchunked version is a multi-GB allocation.
    """
    ab = b - a
    denom = np.maximum((ab * ab).sum(axis=1), 1e-300)
    out = np.empty(len(q))
    for i in range(0, len(q), chunk):
        qq = q[i:i + chunk]
        d = qq[:, None, :] - a[None, :, :]
        t = np.clip((d * ab[None, :, :]).sum(axis=2) / denom[None, :], 0.0, 1.0)
        closest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
        out[i:i + chunk] = np.linalg.norm(qq[:, None, :] - closest, axis=2).min(axis=1)
    return out


def complex_arrays(seg_a, seg_b):
    """Segment endpoint pairs -> (V, E, P) as the potentials want them: welded vertices, an
    index pair per segment, and no isolated points (every vertex here is on a segment)."""
    pts = np.vstack([seg_a, seg_b])
    # Weld on exact coordinates: these came from mesh vertices, so equal points are bit-equal.
    uniq, inv = np.unique(pts, axis=0, return_inverse=True)
    n = len(seg_a)
    E = np.column_stack([inv[:n], inv[n:]]).astype(np.int32)
    E = E[E[:, 0] != E[:, 1]]  # drop any degenerate segment
    return uniq, E, []


def grid(pts, delta, res, pad_rel, pad_abs=None, zoom=1.0):
    """A regular grid over the mesh bbox -> (xs, ys, query points).

    pad_abs overrides the relative padding: the smooth field is identically zero past dhat, so
    for it there is nothing to see further out than that.

    zoom > 1 keeps the same pixel COUNT over a window 1/zoom as wide, centred on the mesh
    bounding box -- so it is a real increase in sampling density, not a crop of an image already
    computed. The padding is symmetric, so the padded box has the same centre as the mesh.
    """
    lo, hi = pts.min(0), pts.max(0)
    pad = pad_abs if pad_abs is not None else pad_rel * np.linalg.norm(hi - lo) + 2.0 * delta
    lo, hi = lo - pad, hi + pad
    if zoom != 1.0:
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo) / zoom
        lo, hi = mid - half, mid + half
    step = max((hi - lo)) / (res - 1)          # square pixels
    xs = np.arange(lo[0], hi[0] + 0.5 * step, step)
    ys = np.arange(lo[1], hi[1] + 0.5 * step, step)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    return xs, ys, np.column_stack([X.ravel(), Y.ravel()])


# ---------------------------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh", type=Path, help="2D .msh")
    ap.add_argument("config", type=Path, nargs="?", help="the run's config, for delta/selection")
    ap.add_argument("--delta", type=float, default=None, help="target_distance; overrides config")
    ap.add_argument("--selection", default=None, help="offset_selection; overrides config")
    ap.add_argument("--res", type=int, default=600, help="pixels on the long axis")
    ap.add_argument("--cap", type=float, default=None,
                    help="clamp the plotted value here before colouring; default c^2 for "
                         "--plot error, 2c for --plot phi. 0 disables.")
    ap.add_argument("--plot", choices=("error", "phi"), default="error",
                    help="error: E = (Phi - c)^2 (default).  phi: the potential itself.")
    ap.add_argument("--cmap", default="viridis",
                    help="a matplotlib colormap name, OR a single color (e.g. '#0090A0') for "
                         "a white -> that-color ramp")
    ap.add_argument("--levels", type=int, default=0,
                    help="draw N level sets of the plotted field. Spaced evenly in sqrt(E) -- "
                         "i.e. in the residual |Phi - c| -- so a quadratic reads evenly instead "
                         "of crowding every line against the level set. 0 (default) draws none")
    ap.add_argument("--pad", type=float, default=0.05, help="bbox padding, relative to diagonal")
    ap.add_argument("--zoom", type=float, default=1.0,
                    help="magnification about the mesh bbox centre. The window narrows by this "
                         "factor while the pixel count stays put, so 2 halves the step -- it "
                         "resamples, it does not crop an existing image.")
    ap.add_argument("--field", choices=("euclidean", "smooth"), default=None,
                    help="which Phi; default is the config's offset_field, else smooth")
    ap.add_argument("--dhat-factor", type=float, default=None,
                    help="smooth field support radius, as a multiple of delta (default 2, or "
                         "the config's offset_dhat_factor)")
    ap.add_argument("--show-input", action="store_true",
                    help="draw the input complex over the pixels")
    ap.add_argument("--include-inside", action="store_true",
                    help="also plot inside the selected region. Off by default: the offset "
                         "grows OUT of that region, so the field in there is not part of the "
                         "problem -- and on a closed input it carries a mirror level set at "
                         "distance delta on the inside that reads as a second offset.")
    ap.add_argument("--save", type=Path, default=None,
                    help="write a PNG here instead of opening a window")
    a = ap.parse_args()

    cfg = json.loads(a.config.read_text()) if a.config else {}
    expr = a.selection or cfg.get("offset_selection") or "!_"
    delta = a.delta if a.delta is not None else cfg.get("target_distance", -1.0)
    prov = "--delta" if a.delta is not None else "config target_distance"
    field = a.field or cfg.get("offset_field", "smooth")
    dhat_factor = (a.dhat_factor if a.dhat_factor is not None
                   else float(cfg.get("offset_dhat_factor", 2.0)))

    pts, tris, groups = read_mesh(a.mesh)
    if delta is None or delta <= 0:
        rel = cfg.get("target_distance_rel", 0.01)
        delta = rel * np.linalg.norm(pts.max(0) - pts.min(0))
        prov = "target_distance_rel %g x bbox diagonal" % rel

    sel = selection_mask(tris, groups, expr)
    seg_a, seg_b, _ = input_segments(pts, tris, sel, expr)
    print("mesh       %s" % a.mesh)
    print("groups     %s" % (", ".join(sorted(groups)) or "(none)"))
    print("selection  %r  ->  %d input segments" % (expr, len(seg_a)))
    print("delta      %g  (%s)" % (delta, prov))

    # THE FIELD. `smooth` goes through the extension module -- the same C++ object the run
    # uses. `euclidean` could be done in numpy, and is, when the module is missing; when it is
    # present both go through it, so the two are the same code path and comparable.
    mod = load_field_module()
    level = delta                 # euclidean: the level IS the distance
    dhat = float("inf")
    pot = None
    if field == "smooth":
        if mod is None:
            sys.exit("--field smooth needs the wmtk_offset_field module. Build it with:\n"
                     "    cmake --build build --target wmtk_offset_field")
        Vc, Ec, _ = complex_arrays(seg_a, seg_b)
        pot = mod.SmoothOffsetPotential2D(Vc, Ec, [], delta, dhat_factor)
        level, dhat = pot.target_level(), pot.dhat()
    elif mod is not None:
        Vc, Ec, _ = complex_arrays(seg_a, seg_b)
        pot = mod.EuclideanOffsetPotential2D(Vc, Ec, [], delta)
        level = pot.target_level()

    pad_abs = None if field == "euclidean" else 1.25 * dhat
    xs, ys, q = grid(pts, delta, a.res, a.pad, pad_abs, a.zoom)
    step = float(xs[1] - xs[0])
    print("field      %s   (level c = %.6g%s)"
          % (field, level, "" if field == "euclidean" else ", dhat = %.6g" % dhat))
    print("grid       %d x %d = %d pixels, step %.5g  ->  delta spans %.1f pixels"
          % (len(xs), len(ys), len(q), step, delta / step))
    print("window     x [%.4g, %.4g]  y [%.4g, %.4g]   zoom %gx"
          % (xs[0], xs[-1], ys[0], ys[-1], a.zoom))

    # Only call the potential where it is nonzero. Outside dhat the smooth field IS zero, so
    # this is exact, and on this kind of model it is most of the grid.
    if pot is None:
        phi = dist_to_segments(q, seg_a, seg_b)
    elif field == "smooth":
        near = dist_to_segments(q, seg_a, seg_b) <= dhat * 1.02
        phi = np.zeros(len(q))
        phi[near] = pot.values(q[near])
        print("           %d of them are within dhat; the potential is evaluated on those"
              % int(near.sum()))
    else:
        phi = pot.values(q)

    if a.plot == "phi":
        vals, label, default_cap = phi, "Phi", 2.0 * level
    else:
        vals, label, default_cap = (phi - level) ** 2, "E = (Phi - c)^2", level * level
    cap = default_cap if a.cap is None else a.cap
    shown = np.array(vals if cap <= 0 else np.minimum(vals, cap), dtype=float)

    # DROP THE INTERIOR. The offset grows out of the selected region, so the field inside it is
    # not part of the problem -- and Phi does not know that: on a closed input curve the level
    # set Phi = c is a PAIR, one at distance delta outside and a mirror one inside, and the
    # inner one is not an offset of anything. Blanked, not clamped, so it cannot be read as a
    # value.
    n_drop = 0
    if not a.include_inside:
        drop = in_selected_region(q, pts, tris, sel)
        shown[drop] = np.nan
        n_drop = int(drop.sum())
        print("interior   %d of %d pixels are inside the selection and are left blank (%.1f%%)"
              % (n_drop, len(q), 100.0 * n_drop / len(q)))

    keep = ~np.isnan(shown)
    if not keep.any():
        sys.exit("every pixel is inside the selected region -- nothing to plot")
    vmin, vmax = float(np.nanmin(shown)), float(np.nanmax(shown))
    n_cap = int((vals[keep] > cap).sum()) if cap > 0 else 0
    print("%-10s min %.6g  max %.6g  (over the plotted pixels)"
          % (a.plot, vals[keep].min(), vals[keep].max()))
    print("colour     [%.6g, %.6g]%s"
          % (vmin, vmax,
             "" if cap <= 0 else "  capped at %.6g (%s); %d pixels clamped (%.1f%%)"
             % (cap, "--cap" if a.cap is not None else "default", n_cap,
                100.0 * n_cap / max(int(keep.sum()), 1))))

    import matplotlib
    if a.save is not None:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # imshow wants (row, col) = (y, x); the grid is raveled i*ny + j with i over x.
    img = shown.reshape(len(xs), len(ys)).T
    extent = (xs[0] - 0.5 * step, xs[-1] + 0.5 * step,
              ys[0] - 0.5 * step, ys[-1] + 0.5 * step)

    # --cmap takes EITHER a registered colormap name OR a single color (any matplotlib color
    # spec: "#0090A0", "teal", "C0"). A single color builds the white -> that-color ramp, so
    # the field's height reads as ink density on the page rather than as a hue journey: zero
    # is the paper, the maximum is the color at full strength, and every value between is one
    # unambiguous step darker than the last.
    #
    # THE BLANKED INTERIOR IS GREY, NOT TRANSPARENT. Under a viridis-like ramp a transparent
    # hole was unmistakable; under a ramp whose LOW end is white it would read as "error ~ 0"
    # -- the exact opposite of "not evaluated here". Neutral grey belongs to no point of the
    # ramp, so it cannot be mistaken for a value.
    if a.cmap in matplotlib.colormaps:
        base = matplotlib.colormaps[a.cmap]
    else:
        import matplotlib.colors as mcolors
        base = mcolors.LinearSegmentedColormap.from_list(
            "white_to_" + a.cmap, ["#ffffff", a.cmap])
    cmap = base.with_extremes(bad=(0.87, 0.87, 0.88, 1.0))
    fig, ax = plt.subplots(figsize=(11, 11 * len(ys) / max(len(xs), 1)), layout="constrained")
    im = ax.imshow(img, origin="lower", extent=extent, cmap=cmap, vmin=vmin,
                   vmax=vmax, interpolation="nearest", aspect="equal")
    # LEVEL SETS, spaced evenly in sqrt(E) rather than in E. E = (Phi - c)^2 is a quadratic in
    # the residual, so evenly-spaced VALUES would put nearly every line out at the rim and none
    # near the offset boundary, which is the part worth reading. Evenly spaced in sqrt(E) is
    # evenly spaced in |Phi - c|: consecutive lines are a constant residual step apart, and the
    # visible bunching where they crowd is the field steepening, not an artefact of the choice.
    # Strictly inside (0, cap) -- a line exactly at the cap would trace the clamp, not the field.
    if a.levels > 0:
        import matplotlib.colors as mcolors
        top = np.sqrt(vmax)
        fracs = np.arange(1, a.levels + 1) / (a.levels + 1)
        lv = (fracs * top) ** 2
        # A darkened member of the ramp itself, so the lines read as the same ink as the fill.
        rgb = np.array(mcolors.to_rgb(base(1.0)))
        cs = ax.contour(xs, ys, img, levels=lv, colors=[tuple(0.45 * rgb)],
                        linewidths=0.55, alpha=0.75)
        ax.clabel(cs, inline=True, fontsize=6, fmt=lambda v: "%.3g" % v)
        print("levels     %d contours at |Phi - c| = %s"
              % (a.levels, ", ".join("%.4g" % np.sqrt(v) for v in lv)))
        print("           (i.e. E = %s)" % ", ".join("%.4g" % v for v in lv))

    if a.show_input:
        from matplotlib.collections import LineCollection
        import matplotlib.colors as _mc
        ink = tuple(0.4 * np.array(_mc.to_rgb(base(1.0))))
        ax.add_collection(LineCollection(np.stack([seg_a, seg_b], axis=1),
                                         colors=[ink], linewidths=0.9))
    # The input outline is drawn from the WHOLE complex, so under --zoom it reaches past the
    # window and matplotlib would autoscale the axes out to meet it -- silently undoing the
    # zoom and leaving unplotted margins. The image's own extent is the frame.
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_title("%s   %s   delta %g, c %.5g%s%s%s"
                 % (a.mesh.name, field, delta, level,
                    "" if cap <= 0 else ", capped at %.4g" % cap,
                    "" if a.zoom == 1.0 else ", zoom %gx" % a.zoom,
                    "" if a.levels <= 0 else
                    "   (contours: E, spaced evenly in |Phi - c|)"), fontsize=10)
    fig.colorbar(im, ax=ax, label=label, shrink=0.85)

    if a.save is not None:
        fig.savefig(a.save, dpi=140)
        print("saved      %s" % a.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
