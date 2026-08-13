#!/usr/bin/env python3
"""Polyscope viewer for topological_offset results.

Every layer is registered separately so it can be toggled in the UI: the input complex, and
per target distance the offset boundary and the offset region itself.

    pip install polyscope numpy
    python3 view_offset.py <outdir> [<outdir> ...]

Each <outdir> is a run_configs.py output directory -- it is scanned for */out.vtu. Pass the
construction and the optimized directory together to compare them in one window:

    python3 view_offset.py plot_con plot_opt

Layers are named "<dir>/<run> offset" and "<dir>/<run> band", plus one "input <tag>" per input
tag. Every layer starts hidden, so the window opens empty and you switch on exactly what you want
to compare. Everything is drawn in the z=0 plane and viewed
orthographically along z, with planar navigation.
"""
import base64
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import polyscope as ps

VTK_DTYPE = {
    "Float64": np.float64,
    "Float32": np.float32,
    "Int64": np.int64,
    "UInt64": np.uint64,
    "Int32": np.int32,
    "UInt8": np.uint8,
}

# One hue per target distance, in the order the runs are found.
PALETTE = [
    (0.12, 0.44, 0.71),
    (0.18, 0.62, 0.49),
    (0.79, 0.54, 0.11),
    (0.71, 0.28, 0.24),
    (0.49, 0.35, 0.66),
    (0.35, 0.35, 0.35),
]


def _decode(el):
    """VTK inline 'binary' is base64 of [UInt64 nbytes][payload], uncompressed."""
    raw = base64.b64decode("".join(el.text.split()))
    nbytes = struct.unpack("<Q", raw[:8])[0]
    return np.frombuffer(raw[8 : 8 + nbytes], dtype=VTK_DTYPE[el.get("type")])


def read_vtu(path):
    """-> (points Nx2, triangles Mx3, {cell array name: values})"""
    piece = ET.parse(path).getroot().find(".//{*}Piece")
    pts = _decode(piece.find("{*}Points")[0]).reshape(-1, 3)[:, :2]

    arrays = {a.get("Name"): a for a in piece.find("{*}Cells")}
    conn = _decode(arrays["connectivity"]).astype(np.int64)
    offs = _decode(arrays["offsets"]).astype(np.int64)
    starts = np.concatenate([[0], offs[:-1]])
    tris = np.array(
        [conn[s:e] for s, e in zip(starts, offs) if e - s == 3], dtype=np.int64
    )

    cell_data = piece.find("{*}CellData")
    cd = {a.get("Name"): _decode(a) for a in cell_data} if cell_data is not None else {}
    return pts, tris, cd


def boundary_edges(tris, inside):
    """Edges incident to exactly one `inside` triangle -- the boundary of that region."""
    count = {}
    for tri, ins in zip(tris, inside):
        if not ins:
            continue
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            key = (a, b) if a < b else (b, a)
            count[key] = count.get(key, 0) + 1
    return np.array([k for k, c in count.items() if c == 1], dtype=np.int64)


def to3d(pts2):
    return np.column_stack([pts2, np.zeros(len(pts2))])


def compact(nodes, edges):
    """Keep only the nodes the edges actually use, and reindex.

    Polyscope draws a sphere at EVERY node of a curve network, so handing it the whole mesh
    vertex array renders thousands of stray dots for vertices no edge touches.
    """
    used = np.unique(edges)
    remap = np.full(len(nodes), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    return nodes[used], remap[edges]


def add_run(label, vtu, color, seen_inputs):
    """Register the offset band, its boundary, and every input tag present as its own layer.

    Which cell array means what is dataset-dependent -- the annots model calls the input
    complex tag_1 and the offset tag_5, the disk example is the other way round -- so nothing
    is hardcoded: `offset_tag` is written by the offset itself and names the band, and every
    other tag_* array is registered as an input layer under its own name.
    """
    pts, tris, cd = read_vtu(vtu)
    nodes = to3d(pts)

    offset = cd.get("offset_tag")
    if offset is None:
        print(f"  {label}: no offset_tag cell array, skipping")
        return seen_inputs
    inside = offset > 0.5

    e = boundary_edges(tris, inside)
    if len(e):
        n_c, e_c = compact(nodes, e)
        net = ps.register_curve_network(f"{label} offset", n_c, e_c, radius=0.0022)
        net.set_color(color)
        net.set_enabled(False)  # everything starts hidden; toggle what you want to compare

    region = ps.register_surface_mesh(
        f"{label} band", nodes, tris[inside], edge_width=0.0
    )
    region.set_color(tuple(min(1.0, c + 0.35) for c in color))
    region.set_enabled(False)  # fills off by default; toggle in the UI

    # Input tags are the same geometry in every run, so register each only once.
    for name, vals in sorted(cd.items()):
        if not name.startswith("tag_") or name in seen_inputs:
            continue
        sel = vals > 0.5
        if not sel.any() or sel.all():
            continue
        ein = boundary_edges(tris, sel)
        if not len(ein):
            continue
        n_i, e_i = compact(nodes, ein)
        inp = ps.register_curve_network(f"input {name}", n_i, e_i, radius=0.0032)
        inp.set_color((0.09, 0.09, 0.11))
        inp.set_enabled(False)
        seen_inputs.add(name)
    return seen_inputs


def main(argv):
    dirs = [Path(a) for a in argv[1:]]
    if not dirs:
        print(__doc__)
        return 1

    ps.init()
    # The data lives in the XY plane, so the camera must look ALONG z with y up. z_up would
    # put the viewpoint in the plane itself and show the mesh edge-on.
    ps.set_up_dir("y_up")
    ps.set_front_dir("z_front")
    ps.set_navigation_style("planar")
    ps.set_view_projection_mode("orthographic")
    ps.set_ground_plane_mode("none")

    seen_inputs = set()
    color_at = 0
    for d in dirs:
        runs = sorted(p for p in d.glob("*/out.vtu"))
        if not runs:
            print(f"{d}: no */out.vtu found")
            continue
        print(f"{d}: {len(runs)} run(s)")
        for vtu in runs:
            label = f"{d.name}/{vtu.parent.name}"
            seen_inputs = add_run(
                label, vtu, PALETTE[color_at % len(PALETTE)], seen_inputs
            )
            color_at += 1

    ps.show()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
