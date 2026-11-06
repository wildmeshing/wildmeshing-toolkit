"""tight_seal_geom_EXPERIMENTAL_mode: the limit of three regions and what "fill" does.

"voronoi" (the default) refuses four listed tags and four as min_regions, and leaves a part
bordering four regions alone. "fill" takes all of those, splits no cell of the part, labels it
with the nearest tags, keeps each tag's topology, and throws if a cell cannot join.

Needs the wildmeshing bindings; no external data."""
import numpy as np
import pytest

import wildmeshing as wm


def _grid_mesh(n, tag_of):
    """[0, n]^2 in unit squares, each cut in two. tag_of(x, y), at a square's centre, gives the
    name of the square's tag, or None for ambient."""
    ids = np.arange((n + 1) ** 2).reshape(n + 1, n + 1)
    V = np.array([(i, j, 0) for j in range(n + 1) for i in range(n + 1)], dtype=float)
    tris, names = [], []
    for j in range(n):
        for i in range(n):
            a, b, c, d = ids[j, i], ids[j, i + 1], ids[j + 1, i + 1], ids[j + 1, i]
            tris += [(a, b, c), (a, c, d)]
            names += [tag_of(i + 0.5, j + 0.5)] * 2
    labels = ["ambient"] + sorted({t for t in names if t})
    values = np.zeros((len(tris), len(labels)), dtype=np.uint8)
    for f, t in enumerate(names):
        values[f, labels.index(t or "ambient")] = 1
    return {
        "V": V,
        "cells": {"triangle": {
            "connectivity": np.array(tris, dtype=int),
            "attributes": {"tags": {"values": values, "labels": labels}},
        }},
    }


def _four_bodies():
    """Four bodies in the corners of [0, 16]^2 around a cross-shaped gap two cells wide."""
    def tag_of(x, y):
        if 7 < x < 9 or 7 < y < 9:
            return None
        return ("a" if y < 8 else "c") if x < 8 else ("b" if y < 8 else "d")
    return _grid_mesh(16, tag_of)


def _slot():
    """Tag a in two bars, with a slot between them that tag b closes at the right."""
    def tag_of(x, y):
        if x > 12:
            return "b"
        return None if 4 < y < 6 else "a"
    return _grid_mesh(16, tag_of)


def _seal(mesh, mode=None, **options):
    opts = {"operation": "tight_seal_geom", "input": ["m"], "tight_seal_geom_distance": 4.0,
            "tight_seal_geom_sanity": True}
    if mode:
        opts["tight_seal_geom_EXPERIMENTAL_mode"] = mode
    opts.update({f"tight_seal_geom_{k}": v for k, v in options.items()})
    outputs, _ = wm.simwild(opts, {"m": mesh})
    return outputs["mesh"]


def _cells(mesh, tag):
    block = mesh["cells"]["triangle"]
    tags = block["attributes"]["tags"]
    column = list(tags["labels"]).index(tag)
    return np.asarray(block["connectivity"])[np.asarray(tags["values"])[:, column] > 0]


def _area(mesh, tag):
    V = np.asarray(mesh["V"])
    P = V[_cells(mesh, tag)]
    a, b = P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]
    return np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum() / 2


def _components_and_euler(cells):
    """Components of the closed union, joined through vertices, and its Euler characteristic."""
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    edges = set()
    for a, b, c in np.sort(cells, axis=1).tolist():
        edges |= {(a, b), (b, c), (a, c)}
        parent[find(b)] = find(a)
        parent[find(c)] = find(a)
    roots = {find(v) for v in list(parent)}
    return len(roots), len(parent) - len(edges) + len(cells)


TAGS = ["a", "b", "c", "d"]


def test_voronoi_refuses_four_listed_tags():
    with pytest.raises(Exception, match="two or three tags"):
        _seal(_four_bodies(), tags=TAGS)


def test_voronoi_refuses_four_min_regions():
    with pytest.raises(Exception, match="min_regions must be 2 or 3"):
        _seal(_four_bodies(), selection="detected", min_regions=4)


def test_voronoi_leaves_a_part_of_four_regions_alone():
    mesh = _four_bodies()
    out = _seal(mesh, selection="detected")
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(_area(mesh, tag))


def test_unknown_mode_refused():
    with pytest.raises(Exception):
        _seal(_four_bodies(), mode="nearest", tags=TAGS)


@pytest.mark.parametrize("options", [
    {"tags": TAGS},
    {"selection": "detected"},
    {"selection": "detected", "min_regions": 4},
])
def test_fill_has_no_limit_of_three_regions(options):
    mesh = _four_bodies()
    out = _seal(mesh, mode="fill", **options)
    # the whole cross goes, each quarter of it to its nearest body
    assert sum(_area(out, tag) for tag in TAGS) == pytest.approx(16 ** 2)
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(16 ** 2 / 4)
        assert _components_and_euler(_cells(out, tag)) == (1, 1)


def test_fill_min_regions_above_what_a_part_borders_fills_nothing():
    mesh = _four_bodies()
    out = _seal(mesh, mode="fill", selection="detected", min_regions=5)
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(_area(mesh, tag))


def test_fill_splits_no_cell_of_the_part():
    mesh = _four_bodies()
    out = _seal(mesh, mode="fill", tags=TAGS)
    assert len(_cells(out, "a")) > len(_cells(mesh, "a"))
    assert len(out["cells"]["triangle"]["connectivity"]) == \
        len(mesh["cells"]["triangle"]["connectivity"])


def test_fill_throws_when_a_cell_would_change_the_topology():
    # filling the slot would join the two bars of tag a
    with pytest.raises(Exception, match="the fill left"):
        _seal(_slot(), mode="fill", tags=["a", "b"])


def test_voronoi_is_still_the_default():
    mesh = _slot()
    default = _seal(mesh, tags=["a", "b"])
    named = _seal(mesh, mode="voronoi", tags=["a", "b"])
    assert np.array_equal(default["V"], named["V"])
    assert np.array_equal(
        default["cells"]["triangle"]["connectivity"], named["cells"]["triangle"]["connectivity"])
