"""Tier 1 — expression grammar, region/filter selection plumbing, spec validation."""
import pytest

from simwild.polyfem_ops import minimum_separation as ms
from simwild.polyfem_ops import spec
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


def test_collision_pairs_dedup_and_pairs():
    # Same side reused across pairs without ids: no user bookkeeping needed.
    unique, pairs = ms._normalize_collision_pairs(
        [[{"region": "A", "filter": "amb"}, {"region": "B", "filter": "amb"}],
         [{"region": "A", "filter": "amb"}, {"region": "C", "filter": "amb"}]])
    assert pairs == [[1, 2], [1, 3]]
    assert len(unique) == 3
    # duplicate pairs collapse
    _, pairs = ms._normalize_collision_pairs(
        [[{"region": "A", "filter": "C"}, "B"],
         [{"region": "A", "filter": "D"}, "B"]])
    assert len(pairs) == 2


def test_collision_pairs_shape_guard():
    for bad in ["ab", {"region": "A"}, [["only-one"]]]:
        with pytest.raises(ValueError):
            ms._normalize_collision_pairs([bad])


def test_minimum_separation_spec():
    R = spec.load_spec("minimum_separation")
    p = spec.validate(R, {"input": "m.msh", "sep": 1e-3,
                          "collision_pairs": [["a", {"region": "b",
                                                     "filter": "amb", "id": 2}]]})
    assert p["scale"] == 0.001 and p["use_laplacian"] is True
    for bad in [
        {"input": "m.msh", "sep": 1e-3, "collision_pairs": [["a"]]},
        {"input": "m.msh", "sep": "big", "collision_pairs": [["a", "b"]]},
        {"input": "m.msh", "sep": 1e-3,
         "collision_pairs": [["a", {"filter": "amb"}]]},          # no region
        {"input": "m.msh", "sep": 1e-3, "collision_pairs": [["a", "b"]],
         "useLaplacian": True},                                   # unknown key
    ]:
        with pytest.raises(ValueError):
            spec.validate(R, bad)


def test_laplacian_smoothing_spec():
    R = spec.load_spec("laplacian_smoothing")
    p = spec.validate(R, {"input": "m.msh",
                          "interfaces": ["tag_2",
                                         {"region": "tag_0", "filter": "tag_1"}]})
    assert p["smooth_positions"] is True
    with pytest.raises(ValueError):   # id not allowed on smoothing interfaces
        spec.validate(R, {"input": "m.msh",
                          "interfaces": [{"region": "tag_0", "id": 1}]})
