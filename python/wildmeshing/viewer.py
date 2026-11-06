"""A polyscope viewer for a chain of components: each stage's outputs, as the memory route returns
them (``outputs`` of ``wildmeshing.<component>(options, inputs)``) or as ``wildmeshing.save`` wrote
them.

    import wildmeshing as wm
    from wildmeshing import viewer
    r1 = wm.tetwild({...}, inputs)[0]
    r2 = wm.topological_offset({"input": "mesh", "envelope": "envelope", ...}, r1)[0]
    viewer.view([("tetwild", r1), ("offset", r2)])

    python -m wildmeshing.viewer <checkpoint stem> [<checkpoint stem> ...]

Per stage, one item per label of its mesh, in three groups: volume tags (its cells carrying the
label: tets, prisms, pyramids, hexes, with the numeric cell and vertex attributes), face tags (its
tracked triangles) and edge tags (its curves). Each item can be shown, shown alone ("only"), and
shown with its envelope -- the rows of the envelope the stage returned that carry the label, off
at start; "ambient", the background, starts hidden. A label keeps its colour in every stage. A slice plane cuts the volumes. Every stage is
moved and scaled alike, so that the whole chain fills the unit box around the origin, z up; each
mesh is placed about its own centre, where its transform gizmo sits.

Needs polyscope (pip install -e ".[viewer]"); the rest of the package does not.
"""
import os
import sys
import zlib

import numpy as np

VOLUME_TYPES = ["tetrahedron", "pyramid", "prism", "hexahedron"]
GROUPS = [("volume tags", VOLUME_TYPES), ("face tags", ["triangle"]), ("edge tags", ["edge"])]
UNTAGGED = "(untagged)"

# Each cell kind's faces, as local vertex indices: triangles and quads.
CELL_FACES = {
    "pyramid": [[0, 3, 2, 1], [0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]],
    "prism": [[0, 2, 1], [3, 4, 5], [0, 1, 4, 3], [1, 2, 5, 4], [2, 0, 3, 5]],
    "hexahedron": [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6],
                   [3, 0, 4, 7]],
}
PALETTE = [(0.89, 0.10, 0.11), (0.22, 0.49, 0.72), (0.30, 0.69, 0.29), (0.60, 0.31, 0.64),
           (1.00, 0.50, 0.00), (0.65, 0.34, 0.16), (0.97, 0.51, 0.75), (0.10, 0.62, 0.62),
           (0.74, 0.74, 0.13), (0.40, 0.40, 0.40)]


def _colour(label):
    """One colour per label name, the same in every stage."""
    return PALETTE[zlib.crc32(label.encode()) % len(PALETTE)]


def _label_rows(block):
    """Per label of `block`'s tags, the rows carrying it; all rows under UNTAGGED without tags."""
    n = len(block["connectivity"])
    tags = block.get("attributes", {}).get("tags")
    if tags is None:
        return {UNTAGGED: np.ones(n, bool)}
    values = np.asarray(tags["values"]) != 0
    out = {name: values[:, j] for j, name in enumerate(tags["labels"])}
    if not values.any(1).all():
        out[UNTAGGED] = ~values.any(1)
    return out


def _scalars(attributes, n, rows=None):
    """The numeric one-column attributes with `n` rows, as float arrays (restricted to `rows`),
    the tags excepted."""
    out = {}
    for name, a in (attributes or {}).items():
        if name == "tags":
            continue
        values = np.asarray(a["values"] if isinstance(a, dict) else a)
        if values.dtype.kind not in "biuf" or values.shape[0] != n:
            continue
        if values.ndim == 2 and values.shape[1] == 1:
            values = values[:, 0]
        if values.ndim == 1:
            out[name] = (values if rows is None else values[rows]).astype(float)
    return out


def _compact(V, C):
    """V restricted to the vertices C uses, and C renumbered onto them."""
    used, local = np.unique(C, return_inverse=True)
    return V[used], local.reshape(C.shape)


def _volume_layers(ps, name, V, mesh, label, colour):
    """The cells of every volume kind carrying `label`: tets as a volume mesh (polyscope 2.6.1
    draws those, but crashed -- bus error, macOS -- on any other cell, a prism or one hex), the
    other kinds as their faces, a face two cells share drawn once."""
    cells, layers = mesh["cells"], []
    for kind in VOLUME_TYPES:
        if kind not in cells or not len(cells[kind]["connectivity"]):
            continue
        block = cells[kind]
        rows = _label_rows(block).get(label)
        if rows is None or not rows.any():
            continue
        C = np.asarray(block["connectivity"], np.int64)
        n = len(C)
        if kind == "tetrahedron":
            s = ps.register_volume_mesh(f"{name} tets", V, tets=C[rows])
            s.set_transparency(0.5)
            for q, values in _scalars(mesh.get("vertex_attributes"), len(V)).items():
                s.add_scalar_quantity(q, values, defined_on="vertices")
            owner = None
        else:
            faces, owner, seen = [], [], set()
            for i, cell in enumerate(C[rows]):
                for f in CELL_FACES[kind]:
                    face = [int(cell[j]) for j in f]
                    key = tuple(sorted(face))
                    if key not in seen:
                        seen.add(key)
                        faces.append(face)
                        owner.append(i)
            s = ps.register_surface_mesh(f"{name} {kind}s", V, faces)
            s.set_transparency(0.5)
        s.set_color(colour)
        s.set_edge_width(1.0)
        for q, values in _scalars(block.get("attributes"), n, rows).items():
            if owner is None:
                s.add_scalar_quantity(q, values, defined_on="cells")
            else:
                s.add_scalar_quantity(q, values[owner], defined_on="faces")
        layers.append(s)
    return layers


def _rows_layers(ps, name, V, block, rows, colour, kind, envelope):
    """Triangles (a surface) or edges (a curve and its vertices) of `block`'s `rows`."""
    C = np.asarray(block["connectivity"], np.int64)[rows]
    if not len(C):
        return []
    P, local = _compact(V, C)
    if kind == "triangle":
        s = ps.register_surface_mesh(name, P, local)
        s.set_color(colour)
        s.set_edge_width(1.0)
        if envelope:
            s.set_transparency(0.3)
        return [s]
    s = ps.register_curve_network(name, P, local)
    s.set_radius(0.002 if not envelope else 0.006, relative=True)
    s.set_color(colour)
    if envelope:
        s.set_transparency(0.3)
        return [s]
    # A curve network has no wireframe: its segments' ends as dots, one segment apart from the next.
    ends = ps.register_point_cloud(name + " vertices", P)
    ends.set_radius(0.0032, relative=True)
    ends.set_color((0.1, 0.1, 0.1))
    return [s, ends]


def _placed(mesh, transform):
    """The mesh's vertices moved and scaled by `transform`, then about their own centre, and that
    centre."""
    V = (np.asarray(mesh["V"], float) - transform[0]) * transform[1]
    if V.shape[1] == 2:
        V = np.hstack([V, np.zeros((len(V), 1))])
    centre = (V.min(0) + V.max(0)) / 2 if len(V) else np.zeros(3)
    return V - centre, centre


def _stage_items(ps, stage, outputs, transform):
    """The stage's items: {group: {label: {"layers": [...], "envelope": [...]}}}."""
    items = {group: {} for group, _ in GROUPS}
    envelope = outputs.get("envelope")
    env_V, env_centre = _placed(envelope, transform) if envelope else (None, None)
    # The mesh only: the other outputs a component returns (surface, edges, offset, ...) are
    # derived from it.
    for mesh_name, mesh in outputs.items():
        if mesh_name != "mesh" or not mesh.get("cells"):
            continue
        V, centre = _placed(mesh, transform)
        prefix = stage if mesh_name == "mesh" else f"{stage}/{mesh_name}"
        for group, kinds in GROUPS:
            labels = []
            for kind in kinds:
                if kind in mesh["cells"] and len(mesh["cells"][kind]["connectivity"]):
                    labels += [l for l in _label_rows(mesh["cells"][kind]) if l not in labels]
            for label in labels:
                colour = _colour(label)
                name = f"{prefix} {group[:-5]} {label}"
                if group == "volume tags":
                    layers = _volume_layers(ps, name, V, mesh, label, colour)
                else:
                    block = mesh["cells"][kinds[0]]
                    layers = _rows_layers(ps, name, V, block, _label_rows(block)[label], colour,
                                          kinds[0], False)
                for s in layers:
                    s.set_position(centre)
                env_layers = []
                if envelope and label != UNTAGGED:
                    for kind in ["triangle", "edge"]:
                        block = envelope["cells"].get(kind)
                        if block is None or not len(block["connectivity"]):
                            continue
                        rows = _label_rows(block).get(label)
                        if rows is not None and rows.any():
                            env_layers += _rows_layers(ps, f"{name} envelope {kind}s", env_V,
                                                       block, rows, colour, kind, True)
                for s in env_layers:
                    s.set_position(env_centre)
                items[group][f"{mesh_name}: {label}" if prefix != stage else label] = {
                    "layers": layers, "envelope": env_layers, "show": label != "ambient",
                    "default": label != "ambient", "env": False}
    # An envelope without labels: one item of its own.
    if envelope and all(not _label_rows(b).keys() - {UNTAGGED} for b in envelope["cells"].values()):
        layers = []
        for kind in ["triangle", "edge"]:
            block = envelope["cells"].get(kind)
            if block is not None and len(block["connectivity"]):
                layers += _rows_layers(ps, f"{stage} envelope {kind}s", env_V, block,
                                       np.ones(len(block["connectivity"]), bool),
                                       (0.5, 0.5, 0.5), kind, True)
        for s in layers:
            s.set_position(env_centre)
        items["face tags"]["envelope (unlabelled)"] = {"layers": [], "envelope": layers,
                                                       "show": False, "default": False,
                                                       "env": False}
    return items


def view(stages, screenshots=None):
    """Show `stages`, a list of (name, outputs) -- outputs as a component returns them on the
    memory route, a mesh-name -> MeshData dict. `screenshots`: a directory to write each stage
    from six directions to instead of opening the window; "mock" registers everything without a
    window, as a self-test (WM_VIEWER_SCREENSHOTS does the same)."""
    import polyscope as ps
    import polyscope.imgui as psim

    screenshots = screenshots or os.environ.get("WM_VIEWER_SCREENSHOTS")
    ps.init("openGL_mock" if screenshots == "mock" else "auto")
    ps.set_up_dir("z_up")
    ps.set_ground_plane_mode("none")
    points = [np.asarray(m["V"], float) for _, outputs in stages for m in outputs.values()]
    points = [p if p.shape[1] == 3 else np.hstack([p, np.zeros((len(p), 1))]) for p in points]
    lo = np.min([p.min(0) for p in points if len(p)], 0)
    hi = np.max([p.max(0) for p in points if len(p)], 0)
    transform = ((lo + hi) / 2, 1.0 / max(np.linalg.norm(hi - lo), 1e-300))

    names = [name for name, _ in stages]
    items = [_stage_items(ps, name, outputs, transform) for name, outputs in stages]
    for name, stage_items in zip(names, items):
        print(name, {g: list(i) for g, i in stage_items.items() if i})
    shown = [i == len(stages) - 1 for i in range(len(stages))]
    gizmos = {"slice": True, "transform": False}

    plane = ps.add_scene_slice_plane()
    plane.set_pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    plane.set_draw_plane(False)

    def every_item():
        for i, stage_items in enumerate(items):
            for group in stage_items.values():
                for item in group.values():
                    yield i, item

    def apply():
        for i, item in every_item():
            for s in item["layers"]:
                s.set_enabled(shown[i] and item["show"])
                s.set_transform_gizmo_enabled(gizmos["transform"] and shown[i] and item["show"])
            for s in item["envelope"]:
                s.set_enabled(shown[i] and item["env"])
        plane.set_draw_widget(gizmos["slice"])

    apply()
    ps.set_view_center_raw((0.0, 0.0, 0.0))
    ps.look_at((1.2, -1.6, 0.9), (0.0, 0.0, 0.0))

    if screenshots and screenshots != "mock":
        os.makedirs(screenshots, exist_ok=True)
        eyes = [(1.2, -1.6, 0.9), (-1.6, -1.2, 0.9), (-1.2, 1.6, 0.9), (1.6, 1.2, 0.9),
                (0.3, -0.4, 2.0), (1.0, -1.4, -0.8)]
        for i, name in enumerate(names):
            shown[:] = [j == i for j in range(len(stages))]
            apply()
            for k, eye in enumerate(eyes):
                ps.look_at(eye, (0.0, 0.0, 0.0))
                ps.screenshot(os.path.join(screenshots, f"{i}_{name}_{k}.png"), transparent_bg=False)
        return

    def callback():
        changed = False
        for i, name in enumerate(names):
            c, shown[i] = psim.Checkbox(f"##stage{i}", shown[i])
            changed |= c
            if c and shown[i]:
                # A stage switched on shows everything in it but the background and envelopes.
                for group_items in items[i].values():
                    for item in group_items.values():
                        item["show"], item["env"] = item["default"], False
            psim.SameLine()
            if psim.TreeNode(f"{i + 1}. {name}"):
                for group, group_items in items[i].items():
                    if not group_items or not psim.TreeNode(f"{group}##{i}"):
                        continue
                    for label, item in group_items.items():
                        key = f"{i}/{group}/{label}"
                        if item["layers"]:
                            c, item["show"] = psim.Checkbox(f"{label}##show{key}", item["show"])
                            changed |= c
                        else:
                            psim.TextUnformatted(label)
                        psim.SameLine()
                        if item["envelope"]:
                            c, item["env"] = psim.Checkbox(f"envelope##env{key}", item["env"])
                            changed |= c
                            psim.SameLine()
                        if psim.Button(f"only##only{key}"):
                            for _, other in every_item():
                                other["show"] = other["env"] = False
                            shown[:] = [j == i for j in range(len(stages))]
                            item["show"] = bool(item["layers"])
                            item["env"] = not item["layers"]
                            changed = True
                    psim.TreePop()
                psim.TreePop()
        psim.Separator()
        c, gizmos["slice"] = psim.Checkbox("slice gizmo", gizmos["slice"])
        changed |= c
        c, gizmos["transform"] = psim.Checkbox("transform gizmos", gizmos["transform"])
        changed |= c
        if changed:
            apply()

    ps.set_user_callback(callback)
    if screenshots == "mock":
        ps.show(3)
    else:
        ps.show()


def main(argv):
    import wildmeshing as wm

    if not argv:
        sys.exit(__doc__)
    view([(os.path.basename(stem), wm.load(stem)) for stem in argv])


if __name__ == "__main__":
    main(sys.argv[1:])
