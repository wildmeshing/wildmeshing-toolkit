"""V_rational: the exact positions of the vertices a run could not round, handed on in memory.

A square (2D) or its extrusion (3D) with an interior vertex P 2^-80 above the edge from (0, 0)
to (3, 1): its rounding lies below that edge and inverts the sliver beside it, so P cannot be
rounded. Given only the rounding, simwild and topological_offset refuse the mesh; given P exactly
(V_rational), a tag operation, manifold_extraction or topological_offset keeps P exact, returns it
in V_rational and warns (UnroundedVerticesWarning), and remeshing from that result rounds every
vertex.

Needs the wildmeshing bindings; no external data."""
import os
import warnings
from fractions import Fraction

import numpy as np
import pytest

import wildmeshing as wm

P = (Fraction(1), Fraction(1, 3) + Fraction(1, 2**80))
# A, (3, 0), B = (3, 1), (3, 3), (0, 3), P; the sliver is (A, B, P).
POINTS = [(0, 0), (3, 0), (3, 1), (3, 3), (0, 3), P]
TRIANGLES = [[0, 1, 2], [0, 2, 5], [5, 2, 3], [5, 3, 4], [0, 5, 4]]
IN_A = [0, 0, 0, 1, 1]


def _orientation(points):
    """The sign of the simplex's signed volume, exactly."""
    e = [[q - p for p, q in zip(points[0], r)] for r in points[1:]]
    if len(e) == 2:
        return np.sign(e[0][0] * e[1][1] - e[0][1] * e[1][0])
    return np.sign(e[0][0] * (e[1][1] * e[2][2] - e[1][2] * e[2][1])
                   - e[0][1] * (e[1][0] * e[2][2] - e[1][2] * e[2][0])
                   + e[0][2] * (e[1][0] * e[2][1] - e[1][1] * e[2][0]))


def _mesh(dim):
    """(the mesh with V the rounding only, the same mesh with V_rational, P's exact rows)."""
    if dim == 2:
        exact = [(Fraction(x), Fraction(y), Fraction(0)) for x, y in POINTS]
        cells, in_a, kind = [list(t) for t in TRIANGLES], IN_A, "triangle"
    else:
        exact = [(Fraction(x), Fraction(y), Fraction(z)) for z in (0, 3) for x, y in POINTS]
        n, cells, in_a, kind = len(POINTS), [], [], "tetrahedron"
        for t, a in zip(TRIANGLES, IN_A):
            i, j, k = sorted(t)
            for tet in ([i, j, k, k + n], [i, j, j + n, k + n], [i, i + n, j + n, k + n]):
                cells.append(tet)
                in_a.append(a)
    for c in cells:
        if _orientation([exact[v][:dim] for v in c]) < 0:
            c[-2], c[-1] = c[-1], c[-2]
        assert _orientation([exact[v][:dim] for v in c]) > 0
    rows = [r for r, p in enumerate(exact) if any(Fraction(float(x)) != x for x in p)]
    rounded = {
        "V": np.array([[float(x) for x in p] for p in exact]),
        "cells": {kind: {
            "connectivity": np.array(cells, dtype=np.int64),
            "attributes": {"tags": {"values": np.array([[1 - a, a] for a in in_a], dtype=np.uint8),
                                    "labels": ["ambient", "a"]}}}}}
    with_exact = dict(rounded, V_rational={
        "rows": np.array(rows, dtype=np.int64),
        "values": np.array([exact[r] for r in rows], dtype=object)})
    return rounded, with_exact, [exact[r] for r in rows]


def _call(f):
    """f's result and the categories of the warnings it raised."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        result = f()
    return result, [w.category for w in caught]


def _replace_tags(mesh, **options):
    return wm.simwild({"operation": "replace_tags", "input": ["m"], "replace_tags_in": ["a"],
                       "replace_tags_out": ["b"], **options}, {"m": mesh})


def _remesh(mesh):
    return wm.simwild({"operation": "remeshing", "input": ["m"]}, {"m": mesh})


def _extract(mesh, **options):
    return wm.manifold_extraction({"input": "m", "tag_selection": "a", **options}, {"m": mesh})


def _offset(mesh, **options):
    return wm.topological_offset({"input": "m", "offsets": [{"selection": "a"}], **options},
                                 {"m": mesh})


def _check_exact(mesh, positions):
    """mesh's V_rational is `positions`, each on the row of V that is its rounding."""
    exact = mesh["V_rational"]
    assert exact["rows"].dtype == np.int64 and exact["values"].dtype == object
    assert sorted(tuple(v) for v in exact["values"]) == sorted(positions)
    for r, v in zip(exact["rows"], exact["values"]):
        assert all(isinstance(x, Fraction) for x in v)
        assert np.array_equal(mesh["V"][r], [float(x) for x in v])


@pytest.mark.parametrize("dim", [2, 3])
def test_the_rounding_alone_is_refused(dim):
    rounded, _, _ = _mesh(dim)
    with pytest.raises(RuntimeError):
        _remesh(rounded)


@pytest.mark.parametrize("dim", [2, 3])
def test_the_offset_refuses_the_rounding_alone(dim):
    rounded, _, _ = _mesh(dim)
    with pytest.raises(RuntimeError, match="different orientations in the input"):
        _offset(rounded)


@pytest.mark.parametrize("dim", [2, 3])
def test_a_tag_operation_hands_the_exact_positions_on_to_remeshing(dim):
    _, with_exact, positions = _mesh(dim)
    (outputs, report), caught = _call(lambda: _replace_tags(with_exact))
    assert caught == [wm.UnroundedVerticesWarning]
    assert not report["all_rounded"] and report["#unrounded"] == len(positions)
    _check_exact(outputs["mesh"], positions)
    (outputs, report), caught = _call(lambda: _remesh(outputs["mesh"]))
    assert caught == []
    assert report["all_rounded"] and report["#unrounded"] == 0
    assert "V_rational" not in outputs["mesh"]


@pytest.mark.parametrize("dim", [2, 3])
def test_manifold_extraction_hands_the_exact_positions_on(dim):
    _, with_exact, positions = _mesh(dim)
    (outputs, report), caught = _call(
        lambda: wm.manifold_extraction({"input": "m", "tag_selection": "a"}, {"m": with_exact}))
    assert caught == [wm.UnroundedVerticesWarning]
    assert not report["all_rounded"] and report["#unrounded"] == len(positions)
    _check_exact(outputs["mesh"], positions)
    (outputs, report), _ = _call(lambda: _remesh(outputs["mesh"]))
    assert report["all_rounded"] and "V_rational" not in outputs["mesh"]


@pytest.mark.parametrize("dim", [2, 3])
def test_topological_offset_hands_the_exact_positions_on(dim):
    _, with_exact, positions = _mesh(dim)
    (outputs, report), caught = _call(lambda: _offset(with_exact))
    assert caught == [wm.UnroundedVerticesWarning]
    assert not report["all_rounded"] and report["#unrounded"] == len(positions)
    _check_exact(outputs["mesh"], positions)
    (outputs, report), _ = _call(lambda: _remesh(outputs["mesh"]))
    assert report["all_rounded"] and "V_rational" not in outputs["mesh"]


@pytest.mark.parametrize("run", [_replace_tags, _extract, _offset],
                         ids=["simwild", "manifold_extraction", "topological_offset"])
def test_throw_on_fail_throws_on_unrounded_output(run):
    _, with_exact, _ = _mesh(3)
    with pytest.raises(RuntimeError, match="Not all vertices rounded"):
        run(with_exact, throw_on_fail=True)


def test_write_mesh_warns_that_the_exact_positions_are_lost(tmp_path):
    _, with_exact, _ = _mesh(2)
    _, caught = _call(lambda: wm.write_mesh(os.path.join(tmp_path, "m.msh"), with_exact))
    assert caught == [wm.UnroundedVerticesWarning]


def test_values_may_be_integers_or_strings():
    _, with_exact, positions = _mesh(2)
    with_exact["V_rational"]["values"] = np.array(
        [[str(x) for x in p] for p in positions], dtype=object)
    (outputs, _), _ = _call(lambda: _replace_tags(with_exact))
    _check_exact(outputs["mesh"], positions)


@pytest.mark.parametrize("bad", [
    {"rows": np.array([17]), "values": np.array([[1, 1, 0]], dtype=object)},
    {"rows": np.array([5, 5]), "values": np.array([[1, 1, 0], [1, 1, 0]], dtype=object)},
    {"rows": np.array([5]), "values": np.array([[1, 1]], dtype=object)},
    {"rows": np.array([5])},
])
def test_invalid_v_rational_is_refused(bad):
    rounded, _, _ = _mesh(2)
    with pytest.raises((RuntimeError, ValueError)):
        _remesh(dict(rounded, V_rational=bad))
