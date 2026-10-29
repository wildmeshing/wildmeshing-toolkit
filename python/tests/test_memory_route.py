"""Memory and file routes of the tagged components, called through the bindings.

- On the same input, every op gives the file route's result (compared
  independently of cell order).
- A .msh output holds the mesh only; the envelope is an entry of the memory
  result, passed on to the next call only when the caller does so.
- save/load checkpoints round-trip exactly, and the PolyFEM ops' reader
  (gmsh) reads them with every cell, the background included.

Needs the wildmeshing bindings and gmsh; no PolyFEM, no external data.
Offsets run without their optimization, to keep the suite fast."""
import numpy as np
import pytest

import wildmeshing as wm


def _box_obj(path, lo, hi):
    """Closed, outward-oriented triangulated box [lo, hi]."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(1, 4, 3), (1, 3, 2), (5, 6, 7), (5, 7, 8), (1, 2, 6), (1, 6, 5),
         (4, 8, 7), (4, 7, 3), (1, 5, 8), (1, 8, 4), (2, 3, 7), (2, 7, 6)]
    path.write_text("".join(f"v {a} {b} {c}\n" for a, b, c in v)
                    + "".join(f"f {a} {b} {c}\n" for a, b, c in f))
    return str(path)


def _square_obj(path, lo, hi):
    """Closed polyline around the square [lo, hi] at z = 0."""
    (x0, y0), (x1, y1) = lo, hi
    path.write_text(f"v {x0} {y0} 0\nv {x1} {y0} 0\nv {x1} {y1} 0\nv {x0} {y1} 0\n"
                    "l 1 2 3 4 1\n")
    return str(path)


S2T = {"eps_rel": 1e-2, "skip_simplify": True}
OFFSET = {"offset_selection": "a", "target_distance": 0.05, "offset_output_tags": ["off"],
          "optimize_offset": False}


@pytest.fixture(scope="module")
def work(tmp_path_factory):
    return tmp_path_factory.mktemp("memory_route")


@pytest.fixture(scope="module")
def boxes(work):
    """Two overlapping boxes `a` and `b`, as input surfaces."""
    return [_box_obj(work / "a.obj", (0, 0, 0), (1, 1, 1)),
            _box_obj(work / "b.obj", (0.5, 0.25, 0.25), (1.5, 1.25, 1.25))]


@pytest.fixture(scope="module")
def m1(work, boxes):
    """Tagged tet mesh of the two boxes, from the file route."""
    wm.simwild({"input": boxes, "input_names": ["a", "b"], "output": str(work / "m1"),
                "input_dir": str(work), **S2T})
    return str(work / "m1.msh")


def _top(m):
    return "tetrahedron" if "tetrahedron" in m["cells"] else "triangle"


def canonical(m):
    """Each top-dimensional cell as (sorted corner coordinates, region names),
    rows sorted: independent of vertex and cell order. Ambient is no region."""
    c = m["cells"][_top(m)]
    tags = c["attributes"]["tags"]
    rows = []
    for cell, member in zip(c["connectivity"], tags["values"]):
        corners = tuple(sorted(tuple(m["V"][v]) for v in cell))
        names = tuple(sorted(l for l, b in zip(tags["labels"], member) if b and l != "ambient"))
        rows.append((corners, names))
    return sorted(rows)


def exact_diff(a, b):
    """Names of every array that differs between two mesh dicts, compared
    exactly, order included."""
    out = []
    if not np.array_equal(a["V"], b["V"]):
        out.append("V")
    for t in sorted(set(a["cells"]) | set(b["cells"])):
        ca, cb = a["cells"].get(t), b["cells"].get(t)
        if ca is None or cb is None:
            out.append(f"cells[{t}] on one side only")
            continue
        if not np.array_equal(ca["connectivity"], cb["connectivity"]):
            out.append(f"cells[{t}]")
        aa, ab = ca.get("attributes", {}), cb.get("attributes", {})
        for k in sorted(set(aa) | set(ab)):
            if k not in aa or k not in ab or not np.array_equal(
                    aa[k]["values"], ab[k]["values"], equal_nan=True) \
                    or list(aa[k]["labels"]) != list(ab[k]["labels"]):
                out.append(f"cells[{t}].{k}")
    va, vb = a.get("vertex_attributes", {}), b.get("vertex_attributes", {})
    for k in sorted(set(va) | set(vb)):
        if k not in va or k not in vb or not np.array_equal(
                va[k]["values"], vb[k]["values"], equal_nan=True):
            out.append(f"vertex {k}")
    return out


# op name -> (component, options), each run on m1
OPS = {
    "remeshing": ("simwild", {"operation": "remeshing", "eps_rel": 1e-2, "stop_energy": 100}),
    "resolve_overlaps": ("simwild", {"operation": "resolve_overlaps",
                                     "resolve_overlaps_tags": [["a", "b"]]}),
    "replace_tags": ("simwild", {"operation": "replace_tags", "replace_tags_in": ["a & b"],
                                 "replace_tags_out": ["b"]}),
    "tag_priority": ("simwild", {"operation": "tag_priority", "tag_priority": ["a", "b"]}),
    "tight_seal_topo": ("simwild", {"operation": "tight_seal_topo",
                                    "tight_seal_tag_sets": [["a", "b"]]}),
    "keep_lcc": ("simwild", {"operation": "keep_lcc", "keep_lcc_tags": ["a"]}),
    "fill_holes_topo": ("simwild", {"operation": "fill_holes_topo", "fill_holes_tags": ["a"]}),
    "topological_offset": ("topological_offset", OFFSET),
}


def _input(component, value):
    # simwild takes a list of inputs, the other components a single one
    return [value] if component == "simwild" else value


def _same_as_file(memory_result, file_msh):
    back = wm.read_mesh(file_msh)
    assert canonical(memory_result["mesh"]) == canonical(back)
    envelope_type = "triangle" if _top(back) == "tetrahedron" else "edge"
    assert envelope_type not in back["cells"]   # files hold the mesh only


@pytest.mark.parametrize("op", sorted(OPS))
def test_op_same_as_file_route(op, m1, work):
    component, options = OPS[op]
    run = getattr(wm, component)
    run({"input": _input(component, m1), "output": str(work / f"f_{op}"),
         "input_dir": str(work), **options})
    out, report = run({"input": _input(component, "m"), **options}, {"m": wm.read_mesh(m1)})
    assert isinstance(report, dict)
    _same_as_file(out, str(work / f"f_{op}.msh"))


def test_mesh_generation_same_as_file_route(m1, boxes):
    out, _ = wm.simwild({"input": ["a", "b"], "input_names": ["a", "b"], **S2T},
                        {"a": wm.read_mesh(boxes[0]), "b": wm.read_mesh(boxes[1])})
    _same_as_file(out, m1)


def test_triwild_same_as_file_route(work):
    objs = [_square_obj(work / "sa.obj", (0, 0), (1, 1)),
            _square_obj(work / "sb.obj", (0.5, 0.25), (1.5, 1.25))]
    options = {"input_names": ["a", "b"], "eps_rel": 1e-2}
    wm.triwild({"input": objs, "output": str(work / "t1"), "input_dir": str(work), **options})
    out, _ = wm.triwild({"input": ["a", "b"], **options},
                        {"a": wm.read_mesh(objs[0]), "b": wm.read_mesh(objs[1])})
    _same_as_file(out, str(work / "t1.msh"))


def test_manifold_extraction_same_as_file_route(work):
    """Boxes `a` and `c` share one edge, so `a | c` is non-manifold there.
    (manifold_extraction crashes on a selection that is already manifold.)"""
    objs = [_box_obj(work / "ea.obj", (0, 0, 0), (1, 1, 1)),
            _box_obj(work / "ec.obj", (1, 1, 0), (2, 2, 1))]
    wm.simwild({"input": objs, "input_names": ["a", "c"], "output": str(work / "e1"),
                "input_dir": str(work), **S2T})
    options = {"tag_selection": "a | c", "fill_tags": []}
    wm.manifold_extraction({"input": str(work / "e1.msh"), "output": str(work / "f_me"),
                            "input_dir": str(work), **options})
    out, _ = wm.manifold_extraction({"input": "m", **options},
                                    {"m": wm.read_mesh(str(work / "e1.msh"))})
    _same_as_file(out, str(work / "f_me.msh"))


def test_envelope_is_passed_on_only_by_the_caller(boxes):
    r1, _ = wm.simwild({"input": ["a", "b"], "input_names": ["a", "b"], **S2T},
                       {"a": wm.read_mesh(boxes[0]), "b": wm.read_mesh(boxes[1])})
    assert "envelope" in r1
    # passed on explicitly: the offset receives it and returns it
    r2, _ = wm.topological_offset({"input": "m", "envelope": "e", **OFFSET},
                                  {"m": r1["mesh"], "e": r1["envelope"]})
    assert "envelope" in r2
    # not passed on: the offset has none
    r3, _ = wm.topological_offset({"input": "m", **OFFSET}, {"m": r1["mesh"]})
    assert "envelope" not in r3
    # and without its optimization the offset does not depend on it
    assert canonical(r2["mesh"]) == canonical(r3["mesh"])
    # tag-changing ops clear it
    r4, _ = wm.simwild({"operation": "resolve_overlaps", "input": ["m"], "envelope": "e",
                        "resolve_overlaps_tags": [["a", "b"]]},
                       {"m": r1["mesh"], "e": r1["envelope"]})
    assert "envelope" not in r4


def test_checkpoint_round_trip(boxes, work):
    r, _ = wm.simwild({"input": ["a", "b"], "input_names": ["a", "b"], **S2T},
                      {"a": wm.read_mesh(boxes[0]), "b": wm.read_mesh(boxes[1])})
    assert "envelope" in r
    stem = str(work / "ckpt" / "m1")
    wm.save(r, stem)
    back = wm.load(stem)
    assert exact_diff(r["mesh"], back["mesh"]) == []
    assert exact_diff(r["envelope"], back["envelope"]) == []
    # resuming from the checkpoint gives exactly the straight-through result
    options = {"operation": "remeshing", "input": ["m"], "eps_rel": 1e-2, "stop_energy": 100}
    a, _ = wm.simwild(options, {"m": r["mesh"]})
    b, _ = wm.simwild(options, {"m": back["mesh"]})
    assert exact_diff(a["mesh"], b["mesh"]) == []
    # a later save without an envelope must not leave the old one behind
    wm.save({"mesh": a["mesh"]}, stem)
    assert "envelope" not in wm.load(stem)


def test_checkpoint_is_read_by_the_polyfem_reader(boxes, work):
    """The PolyFEM ops read .msh through gmsh (mesh_core.TaggedMesh): a saved
    result must open there with every tet, the background in "ambient"."""
    from wildmeshing.polyfem_ops.mesh_core import TaggedMesh
    r, _ = wm.simwild({"input": ["a", "b"], "input_names": ["a", "b"], **S2T},
                      {"a": wm.read_mesh(boxes[0]), "b": wm.read_mesh(boxes[1])})
    stem = str(work / "ckpt" / "for_polyfem")
    wm.save(r, stem)
    tm = TaggedMesh(f"{stem}.msh")
    tags = r["mesh"]["cells"]["tetrahedron"]["attributes"]["tags"]
    assert len(tm.prim_nodes) == len(tags["values"])
    expected = {}
    for member in tags["values"]:
        names = frozenset(l for l, b in zip(tags["labels"], member) if b)
        expected[names] = expected.get(names, 0) + 1
    found = {}
    for names in tm.prim_tags.values():
        found[names] = found.get(names, 0) + 1
    assert found == expected
    assert frozenset(["ambient"]) in found


def test_checkpoint_with_nan_fields_is_read_by_the_polyfem_reader(m1, work):
    """Without its optimization the offset's diagnostics are NaN everywhere: a saved
    result must still open in gmsh and load back exactly."""
    from wildmeshing.polyfem_ops.mesh_core import TaggedMesh
    r, _ = wm.topological_offset({"input": "m", **OFFSET}, {"m": wm.read_mesh(m1)})
    assert any(np.isnan(a["values"]).any() for a in r["mesh"]["vertex_attributes"].values())
    stem = str(work / "ckpt" / "offset")
    wm.save(r, stem)
    tm = TaggedMesh(f"{stem}.msh")
    assert len(tm.prim_nodes) == len(r["mesh"]["cells"]["tetrahedron"]["connectivity"])
    assert exact_diff(r["mesh"], wm.load(stem)["mesh"]) == []


def test_many_regions_are_read_by_the_polyfem_reader(work):
    """gmsh reads each field header into a fixed buffer: the header must not grow with
    the number of regions."""
    from wildmeshing.polyfem_ops.mesh_core import TaggedMesh
    labels = [f"region_{i:02d}" for i in range(30)]
    m = {"V": np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float),
         "cells": {"tetrahedron": {"connectivity": np.array([[0, 1, 2, 3]], np.int64),
                                   "attributes": {"tags": {"values": np.ones((1, 30), np.uint8),
                                                           "labels": labels}}}}}
    path = str(work / "many_regions.msh")
    wm.write_mesh(path, m)
    assert list(TaggedMesh(path).prim_tags.values()) == [frozenset(labels)]
    assert exact_diff(m, wm.read_mesh(path)) == []
