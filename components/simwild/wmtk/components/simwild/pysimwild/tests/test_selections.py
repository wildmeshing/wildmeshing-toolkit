"""Tier 1 — expression grammar, region/filter selection plumbing, spec validation.

The grammar and the id assignment are tested on the Python functions, which
polyfem_sim keeps using. Collision pairs and the spec rules are tested on the C++
engine: its parameters are validated against the same spec.json files, and the
pairs it builds are the ones it writes into the simulation JSON."""
import json
import re

import h5py
import pytest

from conftest import needs_polyfem_ops, run_cpp
from simwild.polyfem_ops.mesh_core import (assign_selection_ids, normalize_selection,
                               parse_expression)


def _ev(expr, tags):
    pred, _ = parse_expression(expr)
    return pred(frozenset(tags))


def test_expression_grammar_matches_cpp_cases():
    # Mirrors wildmeshing-toolkit tests/test_expression_parser.cpp
    e = "A & (B | !C)"
    assert _ev(e, {"A"}) and not _ev(e, {"B"}) and not _ev(e, {"C"})
    assert _ev(e, {"A", "B"}) and not _ev(e, {"A", "C"}) and _ev(e, {"A", "B", "C"})
    e = "A & B & C"
    assert not _ev(e, {"A", "B"}) and _ev(e, {"A", "B", "C"})
    assert _ev("_", set()) and not _ev("_", {"A"})
    e = "!(_ | C)"
    assert _ev(e, {"A"}) and not _ev(e, {"C"}) and not _ev(e, set())
    e = "A | B & C"          # precedence: A | (B & C)
    assert _ev(e, {"A"}) and not _ev(e, {"B"}) and _ev(e, {"B", "C"})


def test_expression_errors():
    for bad in ["a &", "(a", "a b", 5]:
        with pytest.raises(ValueError):
            parse_expression(bad)


def test_normalize_selection():
    assert normalize_selection("tag_0") == ("tag_0", None, None)
    assert normalize_selection(
        {"region": "a", "filter": "b", "id": 3}) == ("a", "b", 3)
    assert normalize_selection({"region": "a"}) == ("a", None, None)
    for bad in [["a", "b"], "  " and {"selection": ["a", "b"]},
                {"region": "a", "bogus": 1}, {"filter": "b"}, 5]:
        with pytest.raises(ValueError):
            normalize_selection(bad)


def test_assign_ids_dedupes_identical_selections():
    unique, ids = assign_selection_ids(
        [{"region": "A", "filter": "amb"},
         "B",
         {"region": "A", "filter": "amb"}])   # duplicate of the first
    assert ids == [1, 2, 1]
    assert len(unique) == 2


def test_assign_ids_reserves_explicit_and_conflicts():
    unique, ids = assign_selection_ids(
        ["A", {"region": "B", "id": 1}, {"region": "C", "id": 1}])
    assert ids == [2, 1, 1]                    # C shares B's body on purpose
    with pytest.raises(ValueError, match="conflicting"):
        assign_selection_ids([{"region": "A", "id": 1},
                              {"region": "A", "id": 2}])
    with pytest.raises(ValueError, match="explicit 'id'"):
        assign_selection_ids(["A"], require_ids=True)


A = {"region": "tag_0", "filter": "ambient"}
B = {"region": "tag_1", "filter": "ambient"}
BOTH = {"region": "tag_0 | tag_1", "filter": "ambient"}


def _polyfem_pairs(mesh, out_dir, pairs):
    """contact.collision_pairs of the simulation JSON the C++ engine writes."""
    run_cpp(mesh, "minimum_separation", out_dir, collision_pairs=pairs, sep=1.5)
    doc = json.loads((out_dir / "sep_input" / "separation.json").read_text())
    return doc["contact"]["collision_pairs"]


@needs_polyfem_ops
def test_collision_pairs_dedup_and_pairs(boxes3d, tmp_path):
    # Same side reused across pairs without ids: no user bookkeeping needed.
    assert _polyfem_pairs(boxes3d, tmp_path / "reuse",
                          [[A, B], [A, BOTH]]) == [[1, 2], [1, 3]]
    # duplicate pairs collapse, in either order
    assert _polyfem_pairs(boxes3d, tmp_path / "dup",
                          [[A, B], [B, A], [A, B]]) == [[1, 2]]
    # a different filter is a different side
    assert _polyfem_pairs(boxes3d, tmp_path / "filters",
                          [[A, B], [{"region": "tag_0"}, B]]) == [[1, 2], [3, 2]]


@needs_polyfem_ops
def test_collision_pairs_shape_guard(boxes3d, tmp_path):
    for bad in ["ab", {"region": "tag_0"}, [["only-one"]]]:
        with pytest.raises(RuntimeError, match="collision_pairs"):
            run_cpp(boxes3d, "minimum_separation", tmp_path,
                    collision_pairs=[bad], sep=1.5)


@needs_polyfem_ops
def test_minimum_separation_spec(boxes3d, tmp_path):
    run_cpp(boxes3d, "minimum_separation", tmp_path / "ok", sep=1e-3,
            collision_pairs=[["tag_0", {"region": "tag_1",
                                        "filter": "ambient", "id": 2}]])
    # The defaults of scale and use_laplacian, as they reach polyfem.
    doc = json.loads((tmp_path / "ok" / "sep_input" / "separation.json").read_text())
    assert doc["geometry"][0]["transformation"]["scale"] == 0.001
    assert [c["data"].rsplit("/", 1)[-1] for c in doc["constraints"]["soft"]] == [
        "interface_constraint.hdf5", "interface_constraint_laplacian.hdf5"]
    # Each is rejected by the spec rule at `pointer`, which the message names.
    for i, (bad, pointer) in enumerate([
        ({"sep": 1e-3, "collision_pairs": [["tag_0"]]}, "/collision_pairs/*"),
        ({"sep": "big", "collision_pairs": [["tag_0", "tag_1"]]}, "/sep"),
        ({"sep": 1e-3, "collision_pairs": [["tag_0", {"filter": "ambient"}]]},
         "/collision_pairs/*/*"),                                 # no region
        ({"sep": 1e-3, "collision_pairs": [["tag_0", "tag_1"]],
          "useLaplacian": True}, "/useLaplacian"),                # unknown key
    ]):
        with pytest.raises(RuntimeError, match=re.escape(f'"{pointer}"')):
            run_cpp(boxes3d, "minimum_separation", tmp_path / f"bad{i}", **bad)


@needs_polyfem_ops
def test_laplacian_smoothing_spec(boxes3d, tmp_path):
    run_cpp(boxes3d, "laplacian_smoothing", tmp_path / "ok",
            interfaces=["tag_1", {"region": "tag_0", "filter": "ambient"}])
    # smooth_positions defaults on: `b` is then -L (scale * rest positions),
    # which the boxes' edges and corners make nonzero; off, it is all zeros.
    laplacian = tmp_path / "ok" / "smooth_input" / "interface_constraint_laplacian.hdf5"
    with h5py.File(laplacian) as f:
        assert f["b"][()].any()
    # id not allowed on smoothing interfaces
    with pytest.raises(RuntimeError, match=re.escape('"/interfaces/*"')):
        run_cpp(boxes3d, "laplacian_smoothing", tmp_path / "bad",
                interfaces=[{"region": "tag_0", "id": 1}])
