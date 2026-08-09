#!/usr/bin/env python3
"""Polyscope viewer for a triwild 2D result: input curves, envelope curves, output mesh.

    ./visualize_triwild.py runs/full/success/10433        # a sweep output directory
    ./visualize_triwild.py input.obj output.msh           # explicit pair
    ./visualize_triwild.py output.msh                     # mesh alone

Needs polyscope, meshio and numpy; see README.md for the venv.

Three layers, each with its own visibility checkbox:

  input curves      the .obj segment network handed to triwild
  envelope curves   the SIMPLIFIED curves, read back out of the .msh
  output mesh       the triangulation, coloured by per-face AMIPS2D energy on request

Given a directory it finds the mesh (`output.msh`, `out.msh`, or the only `*.msh`), then
the input: an `input.obj` beside it, else the path recorded in the run's `config.json`,
which is what makes it work on a sweep `success/<id>/` directory unchanged. The input is
optional -- without it you get the other two layers.

Two things about the data that the obvious implementation gets wrong:

* **The .msh node array is not the mesh.** Gmsh entities own their nodes, so a triwild
  output holds two disjoint blocks: the dim-2 entity's nodes are the mesh vertices, and
  the dim-1 "EnvelopeSurface" entity's nodes are the simplified input curve. Handing the
  whole array to register_surface_mesh draws the mesh plus several hundred unreferenced
  stray vertices. Each entity is compacted onto the nodes it actually uses.

* **An OBJ `l` record is a polyline, not a segment** -- n indices mean n-1 segments. The
  2D dataset uses both forms, so reading one segment per record silently drops most of
  the network on the files that use long polylines.

What is deliberately NOT here: which output EDGES are constrained (tracked to the
curves). The .msh does not tag them, and inferring them by proximity would be a guess
presented as data.
"""

import json
import sys
from pathlib import Path

import meshio
import numpy as np
import polyscope as ps
import polyscope.imgui as psim

# Orange/blue: the pair that stays separable under the common colour vision
# deficiencies, so the two curve networks never rely on a red/green distinction.
C_INPUT = (0.910, 0.525, 0.165)
C_ENVELOPE = (0.169, 0.498, 0.831)
C_MESH = (0.870, 0.870, 0.885)
C_EDGE = (0.380, 0.380, 0.420)


def read_obj_curves(path):
    """Vertices and segments of an .obj holding a segment network."""
    verts, segs = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3]) if len(p) > 3 else 0.0))
            elif line.startswith("l "):
                # OBJ indices are 1-based, and may be negative (relative to the end).
                idx = []
                for tok in line.split()[1:]:
                    i = int(tok.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                segs.extend(zip(idx[:-1], idx[1:]))  # a polyline is n-1 segments
    return (np.asarray(verts, dtype=float).reshape(-1, 3),
            np.asarray(segs, dtype=np.int64).reshape(-1, 2))


def read_msh(path):
    """(mesh points, triangles, envelope points, envelope segments) from a triwild .msh."""
    m = meshio.read(str(path))

    def gather(kind, ncol):
        blocks = [c.data for c in m.cells if c.type == kind]
        return np.vstack(blocks) if blocks else np.zeros((0, ncol), np.int64)

    def compact(cells, ncol):
        """Restrict to the nodes this entity uses -- see the module docstring."""
        if len(cells) == 0:
            return np.zeros((0, 3)), cells
        used = np.unique(cells)
        remap = np.full(len(m.points), -1, np.int64)
        remap[used] = np.arange(len(used))
        return m.points[used], remap[cells].reshape(-1, ncol)

    mp, mt = compact(gather("triangle", 3), 3)
    ep, es = compact(gather("line", 2), 2)
    return mp, mt, ep, es


def amips2d(P, T):
    """Per-triangle AMIPS2D energy and orientation determinant.

    Transcribed from wmtk::AMIPS2D_energy (src/wmtk/utils/AMIPS2D.cpp). The floor is 2,
    an equilateral triangle -- asserted at startup by _selftest, and cross-checked
    against a run's report.json, whose max_energy this reproduces exactly.

    wmtk substitutes MAX_ENERGY = 1e50 for a triangle that is inverted or degenerate,
    which is what the 1e+50 readings in out.log are. Here the raw expression is returned
    and the determinant handed back separately, so an inverted triangle stays visible as
    a number rather than collapsing to a sentinel.
    """
    x0, y0 = P[T[:, 0], 0], P[T[:, 0], 1]
    x1, y1 = P[T[:, 1], 0], P[T[:, 1], 1]
    x2, y2 = P[T[:, 2], 0], P[T[:, 2], 1]
    a, b = 2.0 / 3.0, 4.0 / 3.0
    num = -(
        x0 * (-b * x0 + a * x1 + a * x2)
        + x1 * (a * x0 - b * x1 + a * x2)
        + y0 * (-b * y0 + a * y1 + a * y2)
        + y1 * (a * y0 - b * y1 + a * y2)
        + y2 * (a * y0 + a * y1 - b * y2)
        + x2 * (a * x0 + a * x1 - b * x2)
    )
    c = 0.577350269189626  # 1/sqrt(3); the 1.15470053837925 below is 2/sqrt(3)
    den = (x0 - x1) * (c * y0 + c * y1 - 2 * c * y2) - (y0 - y1) * (c * x0 + c * x1 - 2 * c * x2)
    with np.errstate(divide="ignore", invalid="ignore"):
        return num / den, den


def _selftest():
    P = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, np.sqrt(3) / 2, 0.0]])
    e, _ = amips2d(P, np.array([[0, 1, 2]]))
    assert abs(e[0] - 2.0) < 1e-9, "AMIPS2D transcription is wrong: got %r" % e[0]


def resolve(args):
    """Turn the command line into (obj path or None, msh path)."""
    paths = [Path(a) for a in args]
    if len(paths) == 2:
        obj, msh = paths
        if obj.suffix == ".msh":  # given the other way round
            obj, msh = msh, obj
        return (obj if obj.is_file() else None), msh
    if len(paths) == 1 and paths[0].is_file():
        p = paths[0]
        return (None, p) if p.suffix == ".msh" else (p, p.with_suffix(".msh"))

    d = paths[0] if paths else Path.cwd()
    if not d.is_dir():
        sys.exit("no such file or directory: %s" % d)

    msh = next((c for c in (d / "output.msh", d / "out.msh") if c.is_file()), None)
    if msh is None:
        found = sorted(d.glob("*.msh"))
        if len(found) != 1:
            sys.exit("expected exactly one .msh in %s, found %d" % (d, len(found)))
        msh = found[0]

    obj = d / "input.obj"
    if not obj.is_file():
        # A sweep output directory keeps the input where it came from, not beside the
        # result -- but the config it ran under records the path.
        obj = None
        cfg = d / "config.json"
        if cfg.is_file():
            try:
                inp = json.load(open(cfg)).get("input")
                if isinstance(inp, str):
                    inp = [inp]
                if inp and Path(inp[0]).is_file():
                    obj = Path(inp[0])
            except Exception:
                pass
    return obj, msh


def main():
    _selftest()
    obj, msh = resolve(sys.argv[1:])
    if not msh.is_file():
        sys.exit("missing mesh: %s" % msh)

    iv, iseg = read_obj_curves(obj) if obj else (np.zeros((0, 3)), np.zeros((0, 2), np.int64))
    mp, mt, ep, es = read_msh(msh)
    energy, den = amips2d(mp, mt)

    print("mesh  %s" % msh)
    print("input %s" % (obj if obj else "(not found -- curves layer omitted)"))
    if obj:
        print("  input curves     %6d verts  %6d segments" % (len(iv), len(iseg)))
    print("  envelope curves  %6d verts  %6d segments" % (len(ep), len(es)))
    print("  output mesh      %6d verts  %6d triangles" % (len(mp), len(mt)))
    if len(mt):
        print("  AMIPS2D          min %.4f  median %.4f  max %.6g   (floor 2)"
              % (np.nanmin(energy), np.nanmedian(energy), np.nanmax(energy)))
        print("  inverted or degenerate triangles: %d" % int((den <= 0).sum()))

    ps.init()
    # The mesh lies flat in z = 0, so the eye has to be ON the z axis looking down.
    # "z_up" would instead put the eye IN that plane and show the mesh edge-on as a line
    # -- and under planar navigation, which only pans and zooms, there is then no way to
    # rotate back out of it. y_up + z_front is the standard 2D orientation: x right, y up.
    ps.set_up_dir("y_up")
    ps.set_front_dir("z_front")
    ps.set_view_projection_mode("orthographic")  # 2D data: perspective only distorts it
    ps.set_ground_plane_mode("none")

    mesh = ps.register_surface_mesh("output mesh", mp, mt, color=C_MESH, edge_width=1.0)
    mesh.set_edge_color(C_EDGE)
    if len(mt):
        mesh.add_scalar_quantity("AMIPS2D energy", energy, defined_on="faces", cmap="viridis")
        mesh.add_scalar_quantity("inverted", (den <= 0).astype(float), defined_on="faces",
                                 cmap="reds")

    env = ps.register_curve_network("envelope curves", ep, es, color=C_ENVELOPE, radius=0.0022)
    inp = ps.register_curve_network("input curves", iv, iseg, color=C_INPUT, radius=0.0015)

    # Frame the data explicitly rather than trusting the home-view heuristic: put the eye
    # straight above the centre of everything drawn, far enough back to clear it.
    box = np.vstack([p for p in (mp, ep, iv) if len(p)])
    lo, hi = box.min(axis=0), box.max(axis=0)
    centre = 0.5 * (lo + hi)
    diag = float(np.linalg.norm(hi - lo)) or 1.0

    def top_view():
        ps.set_navigation_style("planar")  # drag pans, scroll zooms; no rotation
        ps.look_at(
            (float(centre[0]), float(centre[1]), float(centre[2] + diag)),
            (float(centre[0]), float(centre[1]), float(centre[2])),
        )

    top_view()
    state = {"input": bool(obj), "envelope": True, "mesh": True, "rotate": False}

    def callback():
        psim.TextUnformatted(msh.parent.name or str(msh))
        psim.Separator()
        for key, label, struct in (
            ("input", "input curves (orange)", inp),
            ("envelope", "envelope curves (blue)", env),
            ("mesh", "output mesh (grey)", mesh),
        ):
            changed, state[key] = psim.Checkbox(label, state[key])
            if changed:
                struct.set_enabled(state[key])
        psim.Separator()
        if psim.Button("top view"):
            state["rotate"] = False
            top_view()
        psim.SameLine()
        changed, state["rotate"] = psim.Checkbox("allow rotation", state["rotate"])
        if changed:
            ps.set_navigation_style("turntable" if state["rotate"] else "planar")

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    main()
