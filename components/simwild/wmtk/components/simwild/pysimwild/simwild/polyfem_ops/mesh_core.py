"""Shared mesh core: tagged-mesh loading, the wmtk expression grammar in
Python (via the wildmeshing bindings), and interface-face selection.

One loader for every consumer (constraints, boundary extraction, reduction):
reads a physical-groups .msh, merges WMTK's duplicate-tet copies (one copy
per tag in a prim's tag-set) into logical prims carrying the unioned name
set, and precomputes face adjacency.

A selection is the boundary of a region, optionally filtered:
{"region": expr, "filter": expr?, "id": int?} (bare string = region). A
face is selected iff its inside cell satisfies `region`, the outside does
not, and — when given — the outside satisfies `filter`. Expressions follow
the wmtk grammar (&, |, !, parentheses; names = physical groups).
Orientation: outward from the region. Interior faces only; the domain
boundary is never selected.
"""
from collections import defaultdict
from itertools import combinations

import gmsh
import numpy as np


# ---------------------------------------------------------------------------
# Tag expressions — delegated to the C++ parser via the wildmeshing bindings
# (see app/pywildmeshing/pywildmeshing.cpp::Expression). One grammar, one
# implementation; only the identifier lexer below lives in Python.
# ---------------------------------------------------------------------------

def _expr_atoms(expr: str):
    """Identifiers appearing in an expression (lexing only, no grammar)."""
    out, cur = set(), ""
    for c in expr:
        if c.isalnum() or c in "_-":
            cur += c
        else:
            if cur:
                out.add(cur)
            cur = ""
    if cur:
        out.add(cur)
    return out


def parse_expression(expr: str):
    """Compile a wmtk-grammar tag expression through the C++ parser.
    Returns (predicate over a set of names, set of names referenced)."""
    if not isinstance(expr, str):
        raise ValueError(
            f"selection expression must be a string like 'tag_0 | tag_1', "
            f"got {expr!r}")
    from wildmeshing import Expression
    names = _expr_atoms(expr) - {"_"}
    try:
        compiled = Expression(expr, sorted(names))
    except RuntimeError as e:
        raise ValueError(f"selection {expr!r}: {e}") from None
    return (lambda tags: compiled.eval(set(tags))), names


# ---------------------------------------------------------------------------
# Tagged-mesh loading
# ---------------------------------------------------------------------------

class TaggedMesh:
    """A physical-groups mesh with WMTK duplicate prims merged.

    coords          : (n, mesh_dim) float64, indexed via node_tag_to_idx.
    prim_nodes      : {prim key: 0-indexed vertex tuple} (tet or triangle).
    prim_tags       : {prim key: frozenset of group names}.
    face_to_prims   : {frozenset of vertex idx: [prim keys]} (face = tri/edge).
    face_repr       : {face key: ordered vertex tuple from first prim seen}.
    names           : {group name: phys tag}.
    """

    def __init__(self, msh_path):
        gmsh.initialize()
        try:
            gmsh.open(str(msh_path))
            node_tags, flat, _ = gmsh.model.mesh.getNodes()
            self.total_n_nodes = len(node_tags)
            int_tags = [int(t) for t in node_tags]
            # polyfem MshReader uses node_id = tag - 1 for contiguous meshes;
            # match it so constraint columns reference the right FE nodes.
            if max(int_tags) != self.total_n_nodes:
                self.node_tag_to_idx = {t: i for i, t in enumerate(sorted(int_tags))}
            else:
                self.node_tag_to_idx = {t: t - 1 for t in int_tags}

            self.mesh_dim = 3 if gmsh.model.getPhysicalGroups(dim=3) else 2
            raw = np.array(flat, dtype=np.float64).reshape(-1, 3)
            self.coords = np.zeros((self.total_n_nodes, self.mesh_dim))
            for i, t in enumerate(int_tags):
                self.coords[self.node_tag_to_idx[t]] = raw[i, :self.mesh_dim]

            elem_type = 4 if self.mesh_dim == 3 else 2
            npp = 4 if self.mesh_dim == 3 else 3

            self.names = {}
            self.prim_tags = defaultdict(set)
            self.prim_nodes = {}
            canonical = {}
            for d, ptag in gmsh.model.getPhysicalGroups(dim=self.mesh_dim):
                name = gmsh.model.getPhysicalName(d, ptag)
                self.names[name] = int(ptag)
                for ent in gmsh.model.getEntitiesForPhysicalGroup(d, ptag):
                    etags, ntags = gmsh.model.mesh.getElementsByType(elem_type, ent)
                    if len(etags) == 0:
                        continue
                    arr = np.array(ntags, dtype=np.int64).reshape(-1, npp)
                    for j, et in enumerate(etags):
                        vt = frozenset(int(v) for v in arr[j])
                        key = canonical.setdefault(vt, int(et))
                        self.prim_tags[key].add(name)
                        if key not in self.prim_nodes:
                            self.prim_nodes[key] = tuple(
                                self.node_tag_to_idx[int(v)] for v in arr[j])
        finally:
            gmsh.finalize()

        self.prim_tags = {k: frozenset(v) for k, v in self.prim_tags.items()}
        nppf = (4 if self.mesh_dim == 3 else 3) - 1
        self.face_to_prims = defaultdict(list)
        self.face_repr = {}
        for key, nodes in self.prim_nodes.items():
            for fn in combinations(nodes, nppf):
                fk = frozenset(fn)
                self.face_to_prims[fk].append(key)
                if fk not in self.face_repr:
                    self.face_repr[fk] = fn

    def centroid(self, prim_key):
        return np.mean([self.coords[v] for v in self.prim_nodes[prim_key]], axis=0)

    def interior_faces(self):
        """Yield (face_key, prim_a, prim_b) for faces with two incident prims."""
        for fk, prims in self.face_to_prims.items():
            if len(prims) == 2:
                yield fk, prims[0], prims[1]


# ---------------------------------------------------------------------------
# Interface selection: boundary of a region, optionally filtered
# ---------------------------------------------------------------------------

def normalize_selection(spec):
    """Normalize one selection spec to (region_expr, filter_expr|None, id|None).
    Accepts a bare region string or {"region": str, "filter": str, "id": int}."""
    if isinstance(spec, str):
        return spec, None, None
    if isinstance(spec, dict) and "region" in spec:
        extra = set(spec) - {"region", "filter", "id"}
        if extra:
            raise ValueError(f"selection {spec!r}: unknown key(s) {sorted(extra)}")
        sid = spec.get("id")
        filt = spec.get("filter")
        if not isinstance(spec["region"], str) or (
                filt is not None and not isinstance(filt, str)):
            raise ValueError(f"selection {spec!r}: region/filter must be strings")
        return spec["region"], filt, (None if sid is None else int(sid))
    raise ValueError(
        f"a selection is a region expression string or {{'region': str, "
        f"'filter': str, 'id': int}}, got {spec!r} (pairs and 'a & b' "
        f"conjunction selections are no longer supported)")


def assign_selection_ids(selections, require_ids=False):
    """Normalize + dedupe selections and assign ids.

    Identical (region, filter) specs collapse to ONE selection with one id
    (conflicting explicit ids on the same selection raise; distinct
    selections may share an explicit id to form one body). Auto ids are
    assigned sequentially, never colliding with explicit ones.

    Returns (unique, ids_per_input): unique = [{"region", "filter", "id"}]
    in first-appearance order; ids_per_input = id for each input spec.
    """
    specs = [normalize_selection(x) for x in selections]
    if require_ids and any(sid is None for _, _, sid in specs):
        raise ValueError("every selection needs an explicit 'id' here")

    by_key: dict = {}
    order = []
    for region, filt, sid in specs:
        key = (region.strip(), filt.strip() if filt is not None else None)
        if key not in by_key:
            by_key[key] = sid
            order.append(key)
        elif sid is not None:
            if by_key[key] is not None and by_key[key] != sid:
                raise ValueError(
                    f"selection region={key[0]!r} filter={key[1]!r} given "
                    f"conflicting ids {by_key[key]} and {sid}")
            by_key[key] = sid

    reserved = {sid for sid in by_key.values() if sid is not None}
    next_id = 1
    for key in order:
        if by_key[key] is None:
            while next_id in reserved:
                next_id += 1
            by_key[key] = next_id
            next_id += 1

    unique = [{"region": k[0], "filter": k[1], "id": by_key[k]} for k in order]
    ids_per_input = [
        by_key[(r.strip(), f.strip() if f is not None else None)]
        for r, f, _ in specs]
    return unique, ids_per_input


def _compile_selection(sel, mesh: TaggedMesh):
    """Compile a normalized selection dict against a mesh, validating names.
    `_` is rejected: on this side every cell carries a group name — write
    `ambient` instead."""
    preds = []
    for part, expr in (("region", sel["region"]), ("filter", sel["filter"])):
        if expr is None:
            preds.append(None)
            continue
        pred, names = parse_expression(expr)
        if "_" in _expr_atoms(expr):
            raise ValueError(
                f"selection {part}={expr!r}: '_' has no meaning here — every "
                f"cell has a group name; write 'ambient' instead")
        unknown = names - set(mesh.names)
        if unknown:
            raise ValueError(
                f"selection {part}={expr!r} references unknown tag(s) "
                f"{sorted(unknown)}; available: {sorted(mesh.names)}")
        preds.append(pred)
    return preds[0], preds[1]


def select_boundary_faces(mesh: TaggedMesh, selections):
    """Select oriented boundary faces for normalized selection dicts.

    A face is selected iff its inside cell satisfies `region`, the outside
    cell does not, and — when given — the outside satisfies `filter`.
    Interior faces only; domain-boundary faces are never selected.

    Returns records {"face": ordered vertex idx tuple, "a_prim": inside key,
    "b_prim": outside key, "ids": sorted id list}. Orientation is the
    consumer's job: outward from the region (A -> B).
    """
    compiled = [(_compile_selection(sel, mesh) + (sel["id"],))
                for sel in selections]

    hits = defaultdict(lambda: defaultdict(set))   # face -> inside prim -> ids
    for fk, p, q in mesh.interior_faces():
        tp, tq = mesh.prim_tags[p], mesh.prim_tags[q]
        for pr, pf, sid in compiled:
            for inside, t_in, t_out in ((p, tp, tq), (q, tq, tp)):
                if pr(t_in) and not pr(t_out) and (pf is None or pf(t_out)):
                    hits[fk][inside].add(sid)

    out = []
    for fk, insides in hits.items():
        prims = mesh.face_to_prims[fk]
        for a_prim, ids in insides.items():
            b_prim = prims[1] if prims[0] == a_prim else prims[0]
            out.append({"face": mesh.face_repr[fk], "a_prim": a_prim,
                        "b_prim": b_prim, "ids": sorted(ids)})
    return out


def select_region_nodes(mesh: TaggedMesh, exprs) -> np.ndarray:
    """Sorted 0-indexed node ids of every cell whose tag-set satisfies any
    of the wmtk Boolean expressions in `exprs` (e.g. ["floor", "rod | anchor"]).
    Raises if an expression selects no cells (almost always a typo'd tag)."""
    ids = set()
    for expr in exprs:
        pred, _ = parse_expression(expr)
        hit = False
        for key, tags in mesh.prim_tags.items():
            if pred(tags):
                hit = True
                ids.update(mesh.prim_nodes[key])
        if not hit:
            raise ValueError(f"region expression {expr!r} selects no cells")
    return np.array(sorted(ids), dtype=np.int64)
