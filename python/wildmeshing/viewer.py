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
at start; "ambient", the background, starts hidden. A label keeps its colour in every stage. A
slice plane, off at start, cuts the volumes. Every stage is moved and scaled alike, so that the
whole chain fills the unit box around the origin, z up; each mesh is placed about its own centre,
where its transform gizmo sits.

Attributes are shown on demand: per stage, its mesh's numeric attributes but the tags, one entry
per column with its range, and one "colour by" choice among them, registered on the stage's layers
only when chosen. Correspondences are drawn on demand too, as a line from each point to its image,
coloured by length: an offset's (save_offset_correspondence: each front vertex to the vertex its
corr_input_vid names) and a surface history's (a stage given with its report and input mesh, the
history in report["history"]: each input vertex to its image on the output, through
wildmeshing.Tracking, and back).

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


def _values(a):
    return np.asarray(a["values"] if isinstance(a, dict) else a)


def _attributes(mesh):
    """Every numeric attribute column of `mesh` but the tags, once: {"label", "kind" ("vertex" or
    the cell kind it lives on), "values" (one per vertex or cell), "range" (the finite values' min
    and max; None when none is finite, so nothing to colour by), "text"}."""
    sources = [("vertex", mesh.get("vertex_attributes"), len(mesh["V"]))]
    sources += [(kind, block.get("attributes"), len(block["connectivity"]))
                for kind, block in mesh.get("cells", {}).items()]
    out = []
    for kind, attributes, n in sources:
        for name, a in (attributes or {}).items():
            values = _values(a)
            if name == "tags" or values.dtype.kind not in "biuf" or not n or values.shape[0] != n:
                continue
            values = values.reshape(n, -1).astype(float)
            for j in range(values.shape[1]):
                column = values[:, j]
                finite = column[np.isfinite(column)]
                bad = n - len(finite)
                value_range = (float(finite.min()), float(finite.max())) if len(finite) else None
                text = f"{value_range[0]:.4g} .. {value_range[1]:.4g}" if value_range else ""
                if bad:
                    text += f"{', ' if text else ''}{bad} of {n} not finite"
                out.append({"label": f"{kind} {name}" + (f"[{j}]" if values.shape[1] > 1 else ""),
                            "kind": kind, "values": column, "range": value_range, "text": text})
    return out


def _colour_by(targets, attribute):
    """Colour each layer in `targets` by `attribute` (None: by its label's colour again) where the
    attribute lives on what the layer draws: a vertex attribute on every layer, a cell attribute
    on the layers of its own cell kind. A layer holds one such quantity at a time."""
    for t in targets:
        if t.get("quantity"):
            t["structure"].remove_quantity(t.pop("quantity"))
        if attribute is None:
            continue
        values = attribute["values"]
        if attribute["kind"] == "vertex":
            values = values if t["used"] is None else values[t["used"]]
            on = "nodes" if t["kind"] == "edge" else "vertices"
        elif attribute["kind"] == t["kind"]:
            values = values[t["rows"]] if t["owner"] is None else values[t["rows"]][t["owner"]]
            on = {"tetrahedron": "cells", "edge": "edges"}.get(t["kind"], "faces")
        else:
            continue
        t["structure"].add_scalar_quantity(attribute["label"], values, defined_on=on,
                                           enabled=True, vminmax=attribute["range"])
        t["quantity"] = attribute["label"]


def _offset_correspondence(mesh):
    """An offset's correspondence (save_offset_correspondence): each vertex whose corr_input_vid
    is set (>= 0: the band's front) to the vertex of that vid. None without these attributes."""
    va = mesh.get("vertex_attributes") or {}
    if "corr_input_vid" not in va or "vid" not in va:
        return None
    corr = _values(va["corr_input_vid"]).ravel()
    row = {int(v): k for k, v in enumerate(_values(va["vid"]).ravel())}
    front = np.flatnonzero(corr >= 0)
    image = np.array([row.get(int(c), -1) for c in corr[front]], np.int64)
    V = np.asarray(mesh["V"], float)
    B = np.full((len(front), V.shape[1]), np.nan)
    B[image >= 0] = V[image[image >= 0]]
    return {"diagonal": np.linalg.norm(V.max(0) - V.min(0)), "warnings": "",
            "directions": [{"name": "front -> input vertex", "A": V[front], "B": B}]}


def _corner_points(F, n):
    """One (face, barycentric) per vertex 0..n-1 of F: the vertex as a corner of one of its
    faces; face -1 for a vertex in no face."""
    face = np.full(n, -1, np.int64)
    corner = np.zeros(n, np.int64)
    for k in range(3):
        face[F[:, k]] = np.arange(len(F))
        corner[F[:, k]] = k
    return face, np.eye(3)[corner]


def _evaluate(V, F, face, bary):
    """The positions of (face, barycentric) points on (V, F); NaN where face is -1."""
    P = np.full((len(face), V.shape[1]), np.nan)
    ok = face >= 0
    P[ok] = np.einsum("nk,nkd->nd", bary[ok], V[F[face[ok]]])
    return P


def _history_correspondence(source, mesh, history):
    """A surface history's correspondence: each vertex of `source` (the stage's input) on the
    output `mesh`, and each output vertex back on `source`, each mapped as a corner of one of its
    faces through wildmeshing.Tracking. Raises ValueError where the history does not name these
    two meshes' triangles."""
    from wildmeshing import Tracking

    tracking = Tracking(history)
    if tracking.input_by_vertices or tracking.output_by_vertices:
        raise ValueError("the history runs through a tet mesh: not drawn")
    ends = []
    for side, m, F in [("input", source, tracking.input_triangles),
                       ("output", mesh, tracking.output_triangles)]:
        given = m.get("cells", {}).get("triangle", {}).get("connectivity")
        if given is None or not np.array_equal(np.asarray(given, np.int64), F):
            raise ValueError(f"the {side} mesh's triangles are not the history's {side} triangles")
        ends.append((np.asarray(m["V"], float), F))
    (V_in, F_in), (V_out, F_out) = ends
    directions = []
    for name, (Va, Fa), (Vb, Fb), mapped in [
            ("input -> output", ends[0], ends[1], tracking.to_output),
            ("output -> input", ends[1], ends[0], tracking.to_input)]:
        face, bary = _corner_points(Fa, len(Va))
        on_face = face >= 0  # vertices in no face are not on the surface: no image
        directions.append({"name": name, "A": Va[on_face],
                           "B": _evaluate(Vb, Fb, *mapped(face, bary))[on_face]})
    warnings = {k: v for k, v in tracking.build_warnings.items() if v and k != "total"}
    return {"diagonal": np.linalg.norm(V_in.max(0) - V_in.min(0)),
            "warnings": f"map warnings: {warnings}" if warnings else "map warnings: none",
            "directions": directions}


def _correspondence_layers(ps, name, built, transform):
    """Each direction of `built` as lines (coloured by length, relative to the diagonal) and the
    images as points, hidden, with its text."""
    for d in built["directions"]:
        ok = ~np.isnan(d["B"]).any(1)
        length = np.linalg.norm(d["A"][ok] - d["B"][ok], axis=1) / built["diagonal"]
        A, B = _world(d["A"][ok], transform), _world(d["B"][ok], transform)
        m = int(ok.sum())
        lines = ps.register_curve_network(f"{name} {d['name']}", np.vstack([A, B]),
                                          np.stack([np.arange(m), np.arange(m) + m], 1),
                                          enabled=False)
        lines.set_radius(0.0006, relative=True)
        lines.add_scalar_quantity("length / diagonal", length, defined_on="edges", enabled=True)
        images = ps.register_point_cloud(f"{name} {d['name']} images", B, enabled=False)
        images.set_radius(0.0015, relative=True)
        d["layers"] = {"lines": lines, "images": images}
        d["text"] = (f"{m} lines, length / diagonal mean {length.mean():.2e}, "
                     f"max {length.max():.2e}" if m else "no lines") + \
            (f", {len(ok) - m} unmapped" if m < len(ok) else "")


def _compact(V, C):
    """V restricted to the vertices C uses, C renumbered onto them, and those vertices."""
    used, local = np.unique(C, return_inverse=True)
    return V[used], local.reshape(C.shape), used


def _volume_layers(ps, name, V, mesh, label, colour, targets):
    """The cells of every volume kind carrying `label`: tets as a volume mesh (polyscope 2.6.1
    draws those, but crashed -- bus error, macOS -- on any other cell, a prism or one hex), the
    other kinds as their faces, a face two cells share drawn once. Each layer joins `targets`, the
    layers an attribute can colour."""
    cells, layers = mesh["cells"], []
    for kind in VOLUME_TYPES:
        if kind not in cells or not len(cells[kind]["connectivity"]):
            continue
        block = cells[kind]
        rows = _label_rows(block).get(label)
        if rows is None or not rows.any():
            continue
        C = np.asarray(block["connectivity"], np.int64)
        if kind == "tetrahedron":
            s = ps.register_volume_mesh(f"{name} tets", V, tets=C[rows])
            s.set_transparency(0.5)
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
        targets.append({"structure": s, "kind": kind, "rows": rows,
                        "owner": None if owner is None else np.asarray(owner, np.int64),
                        "used": None})
        layers.append(s)
    return layers


def _rows_layers(ps, name, V, block, rows, colour, kind, envelope, targets=None):
    """Triangles (a surface) or edges (a curve and its vertices) of `block`'s `rows`; the
    triangles or the curve join `targets` when given."""
    C = np.asarray(block["connectivity"], np.int64)[rows]
    if not len(C):
        return []
    P, local, used = _compact(V, C)
    if kind == "triangle":
        s = ps.register_surface_mesh(name, P, local)
    else:
        s = ps.register_curve_network(name, P, local)
        s.set_radius(0.002 if not envelope else 0.006, relative=True)
    if targets is not None:
        targets.append({"structure": s, "kind": kind, "rows": rows, "owner": None, "used": used})
    s.set_color(colour)
    if kind == "triangle":
        s.set_edge_width(1.0)
    if envelope:
        s.set_transparency(0.3)
    if kind == "triangle" or envelope:
        return [s]
    # A curve network has no wireframe: its segments' ends as dots, one segment apart from the next.
    ends = ps.register_point_cloud(name + " vertices", P)
    ends.set_radius(0.0032, relative=True)
    ends.set_color((0.1, 0.1, 0.1))
    return [s, ends]


def _world(V, transform):
    """Vertices `V` (2D ones at z = 0) moved and scaled by `transform`."""
    V = np.asarray(V, float)
    if V.shape[1] == 2:
        V = np.hstack([V, np.zeros((len(V), 1))])
    return (V - transform[0]) * transform[1]


def _placed(mesh, transform):
    """The mesh's vertices moved and scaled by `transform`, then about their own centre, and that
    centre."""
    V = _world(mesh["V"], transform)
    centre = (V.min(0) + V.max(0)) / 2 if len(V) else np.zeros(3)
    return V - centre, centre


def _stage_items(ps, stage, outputs, transform, targets):
    """The stage's items: {group: {label: {"layers": [...], "envelope": [...]}}}; the mesh's
    layers join `targets`."""
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
                    layers = _volume_layers(ps, name, V, mesh, label, colour, targets)
                else:
                    block = mesh["cells"][kinds[0]]
                    layers = _rows_layers(ps, name, V, block, _label_rows(block)[label], colour,
                                          kinds[0], False, targets)
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


def _correspondences(outputs, report, source):
    """The correspondences a stage can draw, not built yet: {"name", "build" (a function
    returning what _offset_correspondence / _history_correspondence return), "directions",
    "built", "error", "warnings"}."""
    mesh = outputs.get("mesh") or {}
    out = []
    if {"corr_input_vid", "vid"} <= set(mesh.get("vertex_attributes") or {}):
        out.append({"name": "offset correspondence", "names": ["front -> input vertex"],
                    "build": lambda: _offset_correspondence(mesh), "error": None})
    if isinstance((report or {}).get("history"), bytes):
        out.append({"name": "history correspondence",
                    "names": ["input -> output", "output -> input"],
                    "build": lambda: _history_correspondence(source, mesh, report["history"]),
                    "error": None if source is not None else
                    "needs the stage's input mesh: view((name, outputs, report, input))"})
    for entry in out:
        entry.update(directions=[{"name": n, "lines": False, "images": False}
                                 for n in entry.pop("names")], built=False, warnings="")
    return out


def _transform(outputs_list):
    """The move and scale that put every mesh of `outputs_list` in the unit box about the origin."""
    points = [_world(m["V"], (np.zeros(3), 1.0)) for outputs in outputs_list
              for m in outputs.values()]
    points = [p for p in points if len(p)]
    if not points:
        return np.zeros(3), 1.0
    lo, hi = np.min([p.min(0) for p in points], 0), np.max([p.max(0) for p in points], 0)
    return (lo + hi) / 2, 1.0 / max(np.linalg.norm(hi - lo), 1e-300)


OPEN_ALL = False  # the tests draw every tree node open, on polyscope's mock backend


def _node(psim, label):
    if OPEN_ALL:
        psim.SetNextItemOpen(True)
    return psim.TreeNode(label)


class Viewer:
    """The stages shown and their panel: view() sets a fixed list of stages, the pipeline builder
    sets each stage as it runs. Every stage is moved and scaled by one transform -- the first
    stage's, unless one is given."""

    def __init__(self, ps, psim, transform=None):
        self.ps, self.psim, self.transform = ps, psim, transform
        self.stages = []
        # The slice plane off at start: on, it hides half of every volume until moved.
        self.gizmos = {"slice": False, "transform": False}
        self.plane = ps.add_scene_slice_plane()
        self.plane.set_pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        self.plane.set_draw_plane(False)

    def set_stage(self, name, outputs, report=None, source=None, shown=True, replace=None,
                  folder=None):
        """Show the stage `name` -- outputs and report as a component returns them on the memory
        route, source the MeshData it ran on, which a history's correspondence needs -- in place of
        the stage of that name, or of the stage `replace` (a renamed one), if any; in the panel,
        under `folder` if one is given."""
        if self.transform is None:
            self.transform = _transform([outputs])
        old = self.remove_stage(replace or name)
        targets = []
        stage = {"name": name, "shown": shown, "targets": targets, "folder": folder,
                 "items": _stage_items(self.ps, name, outputs, self.transform, targets),
                 "attributes": _attributes(outputs["mesh"])
                 if (outputs.get("mesh") or {}).get("cells") else [],
                 "colour": None, "correspondences": _correspondences(outputs, report, source)}
        self.stages.insert(len(self.stages) if old is None else old, stage)
        self.apply()

    def remove_stage(self, name):
        """Remove the stage `name` and its layers; its index, None if there was none."""
        for i, stage in enumerate(self.stages):
            if stage["name"] == name:
                for item in self._items(stage):
                    for s in item["layers"] + item["envelope"]:
                        s.remove()
                for d in self._directions(stage):
                    for s in d.get("layers", {}).values():
                        s.remove()
                del self.stages[i]
                return i
        return None

    @staticmethod
    def _items(stage):
        for group in stage["items"].values():
            yield from group.values()

    @staticmethod
    def _directions(stage):
        for entry in stage["correspondences"]:
            yield from entry["directions"]

    def apply(self):
        for stage in self.stages:
            for item in self._items(stage):
                for s in item["layers"]:
                    s.set_enabled(stage["shown"] and item["show"])
                    s.set_transform_gizmo_enabled(self.gizmos["transform"] and stage["shown"]
                                                  and item["show"])
                for s in item["envelope"]:
                    s.set_enabled(stage["shown"] and item["env"])
            for d in self._directions(stage):
                for key, s in d.get("layers", {}).items():
                    s.set_enabled(stage["shown"] and d[key])
        self.plane.set_enabled(self.gizmos["slice"])
        self.plane.set_draw_widget(self.gizmos["slice"])

    def _build(self, stage, entry):
        """Build a correspondence the first time one of its layers is asked for (a history's map
        can take seconds)."""
        try:
            built = entry["build"]()
        except (ValueError, RuntimeError) as e:
            entry["error"] = str(e)
            return
        _correspondence_layers(self.ps, f"{stage['name']} {entry['name']}", built, self.transform)
        for d, b in zip(entry["directions"], built["directions"]):
            d.update(layers=b["layers"], text=b["text"])
        entry["warnings"], entry["built"] = built["warnings"], True

    def _show(self, stage, shown):
        stage["shown"] = shown
        if shown:
            # A stage switched on shows everything in it but the background and envelopes.
            for item in self._items(stage):
                item["show"], item["env"] = item["default"], False

    def panel(self):
        """The stages, each a checkbox and a tree: its volume, face and edge tags, attributes and
        correspondences; the stages of one folder under one checkbox and one tree, folded; then
        the gizmo toggles."""
        psim = self.psim
        changed = False
        drawn = set()
        for i, stage in enumerate(self.stages):
            folder = stage["folder"]
            if folder is None:
                changed |= self._stage_row(i, stage)
                continue
            if folder in drawn:
                continue
            drawn.add(folder)
            members = [(j, s) for j, s in enumerate(self.stages) if s["folder"] == folder]
            c, shown = psim.Checkbox(f"##folder{folder}", all(s["shown"] for _, s in members))
            if c:
                for _, s in members:
                    self._show(s, shown)
                changed = True
            psim.SameLine()
            if _node(psim, f"{folder} ({len(members)})##folder{folder}"):
                for j, s in members:
                    changed |= self._stage_row(j, s)
                psim.TreePop()
        psim.Separator()
        c, self.gizmos["slice"] = psim.Checkbox("slice", self.gizmos["slice"])
        changed |= c
        c, self.gizmos["transform"] = psim.Checkbox("transform gizmos", self.gizmos["transform"])
        changed |= c
        if changed:
            self.apply()

    def _stage_row(self, i, stage):
        """One stage's checkbox and tree; whether what is shown changed."""
        psim = self.psim
        name = stage["name"]
        changed, shown = psim.Checkbox(f"##stage{name}", stage["shown"])
        if changed:
            self._show(stage, shown)
        psim.SameLine()
        if not _node(psim, f"{i + 1}. {name}##stage{name}"):
            return changed
        for group, group_items in stage["items"].items():
            if not group_items or not _node(psim, f"{group}##{name}"):
                continue
            for label, item in group_items.items():
                key = f"{name}/{group}/{label}"
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
                    for other_stage in self.stages:
                        other_stage["shown"] = other_stage is stage
                        for other in self._items(other_stage):
                            other["show"] = other["env"] = False
                        for d in self._directions(other_stage):
                            d["lines"] = d["images"] = False
                    item["show"] = bool(item["layers"])
                    item["env"] = not item["layers"]
                    changed = True
            psim.TreePop()
        if stage["attributes"] and _node(psim, f"attributes##{name}"):
            # One choice per stage: polyscope colours a layer by one quantity at a time.
            for k, a in enumerate([None] + stage["attributes"]):
                if a is not None and a["range"] is None:
                    psim.TextDisabled(f"{a['label']}: {a['text']}")
                    continue
                label = "none" if a is None else a["label"]
                if psim.RadioButton(f"{label}##colour{name}/{k}", stage["colour"] is a) \
                        and stage["colour"] is not a:
                    stage["colour"] = a
                    _colour_by(stage["targets"], a)
                if a is not None and a["text"]:
                    psim.SameLine()
                    psim.TextDisabled(a["text"])
            psim.TreePop()
        if stage["correspondences"] and _node(psim, f"correspondence##{name}"):
            for k, entry in enumerate(stage["correspondences"]):
                psim.TextUnformatted(entry["name"])
                if entry["error"]:
                    psim.TextDisabled(entry["error"])
                    continue
                for j, d in enumerate(entry["directions"]):
                    key = f"{name}/{k}/{j}"
                    c_lines, d["lines"] = psim.Checkbox(f"{d['name']}##lines{key}", d["lines"])
                    psim.SameLine()
                    c_images, d["images"] = psim.Checkbox(f"images##images{key}", d["images"])
                    if (c_lines or c_images) and not entry["built"]:
                        self._build(stage, entry)
                    changed |= c_lines or c_images
                    if "text" in d:
                        psim.TextDisabled(d["text"])
                if entry["warnings"]:
                    psim.TextDisabled(entry["warnings"])
            psim.TreePop()
        psim.TreePop()
        return changed


def start(ps, screenshots=None):
    """polyscope, started as every view of a chain is: z up, no ground, its own windows built by
    polyscope_panels(); `screenshots` "mock" (or WM_VIEWER_SCREENSHOTS) without a window."""
    screenshots = screenshots or os.environ.get("WM_VIEWER_SCREENSHOTS")
    ps.init("openGL_mock" if screenshots == "mock" else "auto")
    ps.set_up_dir("z_up")
    ps.set_ground_plane_mode("none")
    ps.set_build_default_gui_panels(False)
    return screenshots


def polyscope_panels(ps, psim):
    """polyscope's own windows, each frame: its settings and its list of every structure folded
    at start -- the stages' panel lists the same layers, by stage and tag -- and the selection."""
    psim.SetNextWindowCollapsed(True, psim.ImGuiCond_Once)
    ps.build_polyscope_gui()
    psim.SetNextWindowCollapsed(True, psim.ImGuiCond_Once)
    ps.build_structure_gui()
    ps.build_pick_gui()


def view(stages, screenshots=None):
    """Show `stages`, a list of (name, outputs), (name, outputs, report) or (name, outputs,
    report, input) -- outputs and report as a component returns them on the memory route, the
    outputs a mesh-name -> MeshData dict; input the MeshData the stage ran on, which a history's
    correspondence needs. `screenshots`: a directory to write each stage from six directions to
    instead of opening the window; "mock" registers everything without a window, as a self-test
    (WM_VIEWER_SCREENSHOTS does the same)."""
    import polyscope as ps
    import polyscope.imgui as psim

    stages = [tuple(stage) + (None,) * (4 - len(stage)) for stage in stages]
    screenshots = start(ps, screenshots)
    viewer = Viewer(ps, psim, _transform([outputs for _, outputs, _, _ in stages]))
    for i, (name, outputs, report, source) in enumerate(stages):
        viewer.set_stage(name, outputs, report, source, shown=i == len(stages) - 1)
        print(name, {g: list(v) for g, v in viewer.stages[-1]["items"].items() if v})
    ps.set_view_center_raw((0.0, 0.0, 0.0))
    ps.look_at((1.2, -1.6, 0.9), (0.0, 0.0, 0.0))

    if screenshots and screenshots != "mock":
        os.makedirs(screenshots, exist_ok=True)
        eyes = [(1.2, -1.6, 0.9), (-1.6, -1.2, 0.9), (-1.2, 1.6, 0.9), (1.6, 1.2, 0.9),
                (0.3, -0.4, 2.0), (1.0, -1.4, -0.8)]
        for i, stage in enumerate(viewer.stages):
            for other in viewer.stages:
                other["shown"] = other is stage
            viewer.apply()
            for k, eye in enumerate(eyes):
                ps.look_at(eye, (0.0, 0.0, 0.0))
                ps.screenshot(os.path.join(screenshots, f"{i}_{stage['name']}_{k}.png"),
                              transparent_bg=False)
        return

    ps.set_user_callback(lambda: (polyscope_panels(ps, psim), viewer.panel()))
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
