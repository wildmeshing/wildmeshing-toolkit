"""Memory route of the WMTK wrappers (output=None).

- Each op gives the file route's result on the same input. Compared
  order-independently: the file writers group cells by physical group, so
  only the cell order differs.
- A memory result has an "envelope" exactly when the file route writes one.
- save/load checkpoints round-trip exactly, and an op on a loaded
  checkpoint equals the op on the original.

Needs the wildmeshing bindings; no PolyFEM, no external data."""
import numpy as np
import pytest

from pywmtk import simwild as wm


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


@pytest.fixture(scope="module")
def work(tmp_path_factory):
    return tmp_path_factory.mktemp("memory_route")


@pytest.fixture(scope="module")
def overlapping_boxes(work):
    """Two overlapping boxes `a` and `b`, as input surfaces."""
    return [_box_obj(work / "a.obj", (0, 0, 0), (1, 1, 1)),
            _box_obj(work / "b.obj", (0.5, 0.25, 0.25), (1.5, 1.25, 1.25))]


@pytest.fixture(scope="module")
def m1(work, overlapping_boxes):
    """Tagged tet mesh of the two boxes, from the file route."""
    wm.surf_to_tet(meshes=overlapping_boxes, tags=["a", "b"],
                   output=str(work / "m1"), **S2T)
    return str(work / "m1.msh")


@pytest.fixture(scope="module")
def edge_touching(work):
    """Boxes `a` and `c` sharing one edge: `a | c` is non-manifold there.
    (manifold_extraction crashes on a selection that is already manifold.)"""
    objs = [_box_obj(work / "ea.obj", (0, 0, 0), (1, 1, 1)),
            _box_obj(work / "ec.obj", (1, 1, 0), (2, 2, 1))]
    wm.surf_to_tet(meshes=objs, tags=["a", "c"], output=str(work / "e1"), **S2T)
    return str(work / "e1.msh")


def _top(m):
    return "tetrahedron" if "tetrahedron" in m["cells"] else "triangle"


def canonical(m):
    """Each top-dimensional cell as (sorted corner coordinates, region
    names), rows sorted: independent of vertex and cell order. Ambient is
    no region."""
    c = m["cells"][_top(m)]
    tags = c["attributes"]["tags"]
    labels = list(tags["labels"])
    rows = []
    for cell, member in zip(c["connectivity"], tags["values"]):
        corners = tuple(sorted(tuple(m["V"][v]) for v in cell))
        names = tuple(sorted(l for l, b in zip(labels, member) if b and l != "ambient"))
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


# op name -> (wrapper, kwargs). Each runs on m1 (or edge_touching).
OPS = {
    "remeshing": (wm.remeshing, dict(eps_rel=1e-2, stop_energy=100)),
    "resolve_overlaps": (wm.resolve_overlaps, dict(tags=[["a", "b"]])),
    "replace_tags": (wm.replace_tags, dict(tags_in=["a & b"], tags_out=["b"])),
    "tag_priority": (wm.tag_priority, dict(tags=["a", "b"])),
    "tight_seal_topo": (wm.tight_seal_topo, dict(tags=[["a", "b"]])),
    "keep_lcc": (wm.keep_lcc, dict(tags=["a"])),
    "fill_holes_topo": (wm.fill_holes_topo, dict(tags=["a"])),
    "topological_offset": (wm.topological_offset, dict(
        offset_selection="a", target_distance=0.05, offset_output_tags=["off"],
        others={"optimize_offset": False})),
}


def _check_same_as_file(mem, file_msh):
    outputs, report = mem
    back = wm.read_mesh(file_msh)
    assert canonical(outputs["mesh"]) == canonical(back)
    envelope_type = "triangle" if _top(back) == "tetrahedron" else "edge"
    assert ("envelope" in outputs) == (envelope_type in back["cells"])
    assert isinstance(report, dict)


@pytest.mark.parametrize("op", sorted(OPS))
def test_op_same_as_file_route(op, m1, work):
    fn, kw = OPS[op]
    fn(mesh=m1, output=str(work / f"f_{op}"), **kw)
    _check_same_as_file(fn(mesh=m1, output=None, **kw), str(work / f"f_{op}.msh"))


def test_surf_to_tet_same_as_file_route(m1, overlapping_boxes):
    _check_same_as_file(
        wm.surf_to_tet(meshes=overlapping_boxes, tags=["a", "b"], output=None, **S2T), m1)


def test_lines_to_tri_same_as_file_route(work):
    objs = [_square_obj(work / "sa.obj", (0, 0), (1, 1)),
            _square_obj(work / "sb.obj", (0.5, 0.25), (1.5, 1.25))]
    wm.lines_to_tri(meshes=objs, tags=["a", "b"], output=str(work / "t1"), eps_rel=1e-2)
    _check_same_as_file(
        wm.lines_to_tri(meshes=objs, tags=["a", "b"], output=None, eps_rel=1e-2),
        str(work / "t1.msh"))


def test_manifold_extraction_same_as_file_route(edge_touching, work):
    kw = dict(tag_selection="a | c", radius=0.05)
    wm.manifold_extraction(mesh=edge_touching, output=str(work / "f_me"), **kw)
    _check_same_as_file(wm.manifold_extraction(mesh=edge_touching, output=None, **kw),
                        str(work / "f_me.msh"))


def test_memory_chain_forwards_the_envelope(overlapping_boxes):
    """Results pass straight on, the envelope with them; it is kept or
    dropped where the file route keeps or drops it."""
    r1, _ = wm.surf_to_tet(meshes=overlapping_boxes, tags=["a", "b"], output=None, **S2T)
    r2, _ = wm.remeshing(mesh=r1, eps_rel=1e-2, stop_energy=100, keep_envelope=True,
                         output=None)
    r3, _ = wm.topological_offset(mesh=r2, offset_selection="a", target_distance=0.05,
                                  offset_output_tags=["off"], output=None,
                                  others={"optimize_offset": False})
    r4, _ = wm.remeshing(mesh=r3, eps_rel=1e-2, stop_energy=100, output=None)
    r5, _ = wm.resolve_overlaps(mesh=r1, tags=[["a", "b"]], output=None)
    assert "envelope" in r1 and "envelope" in r2
    assert "envelope" in r3              # received r2's envelope and keeps it
    assert "envelope" not in r4          # remeshing passes write_envelope=False
    assert "envelope" not in r5          # tag-changing ops clear the envelope
    assert len(r4["mesh"]["cells"]["tetrahedron"]["connectivity"]) > 0


def test_checkpoint_round_trip(overlapping_boxes, work):
    r, _ = wm.surf_to_tet(meshes=overlapping_boxes, tags=["a", "b"], output=None, **S2T)
    assert "envelope" in r
    stem = str(work / "ckpt" / "m2")
    wm.save(r, stem)
    back = wm.load(stem)
    assert exact_diff(r["mesh"], back["mesh"]) == []
    assert exact_diff(r["envelope"], back["envelope"]) == []
    # resuming from the checkpoint gives exactly the straight-through result
    a, _ = wm.remeshing(mesh=r, eps_rel=1e-2, stop_energy=100, output=None)
    b, _ = wm.remeshing(mesh=back, eps_rel=1e-2, stop_energy=100, output=None)
    assert exact_diff(a["mesh"], b["mesh"]) == []
    # a later save without an envelope must not leave the old one behind
    wm.save(a, stem)
    assert "envelope" not in wm.load(stem)


def test_memory_result_with_output_path_raises(m1, work):
    r, _ = wm.resolve_overlaps(mesh=m1, tags=[["a", "b"]], output=None)
    with pytest.raises(ValueError, match="output=None"):
        wm.remeshing(mesh=r, output=str(work / "never"))


@pytest.mark.parametrize("removed", [{"rel_ball_threshold": 0.01}, {"overwrite_tags": True}])
def test_topological_offset_rejects_removed_options(m1, removed):
    """Options C++ no longer has fail loudly instead of being dropped."""
    with pytest.raises(TypeError):
        wm.topological_offset(mesh=m1, offset_selection="a", target_distance=0.05,
                              offset_output_tags=[], output=None, **removed)
