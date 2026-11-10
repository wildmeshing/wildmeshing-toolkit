"""tight_seal_geom_EXPERIMENTAL_mode: the limit of three regions, and what "split_fill" does.

"voronoi" (the default) refuses four listed tags and four as min_regions, and leaves a part
bordering four regions alone. "split_fill" takes all of those: it splits each part along the
regions nearest its cells' corners, a cell sharing a facet with a region labelling that facet's
corners with it, and labels each resulting cell from its corners, with no topology test. Both
throw if the join region touches the mesh boundary, even at a vertex.
tight_seal_geom_spatial_selection is on hold: anything but its default throws.

Needs the wildmeshing bindings; no external data."""
import itertools

import numpy as np
import pytest

import wildmeshing as wm


def _grid_mesh(nx, ny, tag_of):
    """[0, nx] x [0, ny] in unit squares, each cut in two along a diagonal that alternates from
    square to square. tag_of(x, y), at a face's centroid, gives the name of its tag, or None for
    ambient."""
    ids = np.arange((nx + 1) * (ny + 1)).reshape(ny + 1, nx + 1)
    V = np.array([(i, j, 0) for j in range(ny + 1) for i in range(nx + 1)], dtype=float)
    tris = []
    for j in range(ny):
        for i in range(nx):
            a, b, c, d = ids[j, i], ids[j, i + 1], ids[j + 1, i + 1], ids[j + 1, i]
            tris += [(a, b, c), (a, c, d)] if (i + j) % 2 == 0 else [(a, b, d), (b, c, d)]
    tris = np.array(tris, dtype=int)
    names = [tag_of(*c) for c in V[tris][:, :, :2].mean(axis=1).tolist()]
    labels = ["ambient"] + sorted({t for t in names if t})
    values = np.zeros((len(tris), len(labels)), dtype=np.uint8)
    for f, t in enumerate(names):
        values[f, labels.index(t or "ambient")] = 1
    return {
        "V": V,
        "cells": {"triangle": {
            "connectivity": tris,
            "attributes": {"tags": {"values": values, "labels": labels}},
        }},
    }


def _four_bodies():
    """Four bars in a pinwheel around a square hole, two cells of ambient around them."""
    def tag_of(x, y):
        if 3 < x < 5 and 3 < y < 5 or not (2 < x < 6 and 2 < y < 6):
            return None
        if y > 5 and x < 5:
            return "a"
        if x > 5 and y > 3:
            return "b"
        return "c" if y < 3 else "d"
    return _grid_mesh(8, 8, tag_of)


def _slot():
    """Two bars of tag a over a slot that tag b closes at both ends, ambient around them."""
    def tag_of(x, y):
        if not (2 < x < 8 and 2 < y < 6):
            return None
        if 3 < y < 5:
            return None if 3 < x < 7 else "b"
        return "a"
    return _grid_mesh(10, 8, tag_of)


def _gap_with_another_tag():
    """Bodies a and b with a gap between them that a band of tag c crosses, ambient around."""
    def tag_of(x, y):
        if not (2 < x < 8 and 2 < y < 8):
            return None
        if x < 4:
            return "a"
        if x > 6:
            return "b"
        return "c" if 4 < y < 5 else None
    return _grid_mesh(10, 10, tag_of)


def _grid_mesh_3d(nx, ny, nz, tag_of):
    """[0, nx] x [0, ny] x [0, nz] in unit cubes, each cut into six tets around its diagonal, every
    tet positively oriented. tag_of(x, y, z), at a tet's centroid, gives its tag or None."""
    ids = np.arange((nx + 1) * (ny + 1) * (nz + 1)).reshape(nz + 1, ny + 1, nx + 1)
    V = np.array([(i, j, k) for k in range(nz + 1) for j in range(ny + 1) for i in range(nx + 1)],
                 dtype=float)
    tets = []
    for k, j, i in itertools.product(range(nz), range(ny), range(nx)):
        for axes in itertools.permutations(range(3)):
            corner = [0, 0, 0]
            tet = [ids[k, j, i]]
            for axis in axes:
                corner[axis] = 1
                tet.append(ids[k + corner[2], j + corner[1], i + corner[0]])
            tets.append(tet)
    tets = np.array(tets, dtype=int)
    P = V[tets]
    negative = np.einsum("ij,ij->i", np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]),
                         P[:, 3] - P[:, 0]) < 0
    tets[negative, 2], tets[negative, 3] = tets[negative, 3].copy(), tets[negative, 2].copy()
    names = [tag_of(*c) for c in V[tets].mean(axis=1).tolist()]
    labels = ["ambient"] + sorted({t for t in names if t})
    values = np.zeros((len(tets), len(labels)), dtype=np.uint8)
    for t, name in enumerate(names):
        values[t, labels.index(name or "ambient")] = 1
    return {"V": V, "cells": {"tetrahedron": {
        "connectivity": tets, "attributes": {"tags": {"values": values, "labels": labels}}}}}


def _two_bodies_3_apart(dim):
    """Bodies a (2 < x < 4) and b (7 < x < 9), mirror images about x = 5.5, ambient around."""
    if dim == 2:
        return _grid_mesh(11, 10, lambda x, y: (
            None if not 2 < y < 8 else "a" if 2 < x < 4 else "b" if 7 < x < 9 else None))
    return _grid_mesh_3d(11, 6, 6, lambda x, y, z: (
        None if not (2 < y < 4 and 2 < z < 4) else "a" if 2 < x < 4 else "b" if 7 < x < 9
        else None))


def _tagged(mesh, tag):
    """Each cell with `tag`, as its corners' positions."""
    kind = "tetrahedron" if "tetrahedron" in mesh["cells"] else "triangle"
    block = mesh["cells"][kind]
    column = list(block["attributes"]["tags"]["labels"]).index(tag)
    cells = np.asarray(block["connectivity"])[
        np.asarray(block["attributes"]["tags"]["values"])[:, column] > 0]
    return np.asarray(mesh["V"])[cells]


def _gap_to_the_boundary():
    """Two bodies with a gap between them that runs across the whole mesh."""
    return _grid_mesh(8, 8, lambda x, y: "a" if x < 3 else ("b" if x > 5 else None))


def _diamond_on_the_boundary():
    """Tags a and b fill the mesh but for a diamond, which reaches its boundary at one vertex."""
    return _grid_mesh(
        16,
        8,
        lambda x, y: None if abs(x - 8) + abs(y - 2) < 2 else ("a" if x < 8 else "b"))


def _seal(mesh, mode=None, **options):
    opts = {"operation": "tight_seal_geom", "input": ["m"], "tight_seal_geom_distance": 2.5,
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
def test_split_fill_has_no_limit_of_three_regions(options):
    mesh = _four_bodies()
    out = _seal(mesh, mode="split_fill", **options)
    # the whole hole goes, each side of it to the bar beside it
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(_area(mesh, tag) + 1)
        assert _components_and_euler(_cells(out, tag)) == (1, 1)


def test_split_fill_min_regions_above_what_a_part_borders_fills_nothing():
    mesh = _four_bodies()
    out = _seal(mesh, mode="split_fill", selection="detected", min_regions=5)
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(_area(mesh, tag))


@pytest.mark.parametrize("mode", ["voronoi", "split_fill"])
@pytest.mark.parametrize("mesh", [_gap_to_the_boundary, _diamond_on_the_boundary])
def test_join_region_on_the_mesh_boundary_throws(mesh, mode):
    with pytest.raises(Exception, match="touches the mesh boundary"):
        _seal(mesh(), mode=mode, selection="detected", distance=4.0)


def test_part_left_alone_on_the_mesh_boundary_throws_too():
    # a body one cell from the wall: the strip between borders one region and would not be filled
    def tag_of(x, y):
        if 2 < y < 6:
            return "a" if 1 < x < 4 else ("b" if 8 < x < 14 else None)
        return None
    with pytest.raises(Exception, match=r"touches the mesh boundary.* 0 of them in parts to fill"):
        _seal(_grid_mesh(16, 8, tag_of), selection="detected")


def test_spatial_selection_default_keeps_everything():
    mesh = _gap_with_another_tag()
    default = _seal(mesh, tags=["a", "b"])
    everything = _seal(mesh, tags=["a", "b"], spatial_selection="_|!_")
    assert np.array_equal(default["V"], everything["V"])
    assert np.array_equal(
        default["cells"]["triangle"]["connectivity"], everything["cells"]["triangle"]["connectivity"])
    assert np.array_equal(
        default["cells"]["triangle"]["attributes"]["tags"]["values"],
        everything["cells"]["triangle"]["attributes"]["tags"]["values"])


@pytest.mark.parametrize("selection", ["_", "_ & !_", "c", "!_ | _"])
def test_spatial_selection_is_on_hold(selection):
    with pytest.raises(Exception, match="on hold"):
        _seal(_gap_with_another_tag(), tags=["a", "b"], spatial_selection=selection)


@pytest.mark.parametrize("dim", [2, 3])
def test_split_fill_follows_the_bisector(dim):
    mesh = _two_bodies_3_apart(dim)
    out = _seal(mesh, mode="split_fill", tags=["a", "b"], distance=3.5)
    a = _tagged(out, "a")[:, :, 0]
    b = _tagged(out, "b")[:, :, 0]
    filled_a = a[a.max(axis=1) > 4 + 1e-9]  # the cells of a beyond its body
    filled_b = b[b.min(axis=1) < 7 - 1e-9]
    assert len(filled_a) > 0 and len(filled_b) > 0
    # the fill of each body stops at the bisector, to the precision of the split's bisection
    assert filled_a.max() <= 5.5 + 1e-5
    assert filled_b.min() >= 5.5 - 1e-5


def _region_facets(mesh, tags):
    """The facets (edges in 2D, faces in 3D) of each tag's own cells, as sets of corner positions,
    and those cells themselves."""
    kind = "tetrahedron" if "tetrahedron" in mesh["cells"] else "triangle"
    block = mesh["cells"][kind]
    labels = list(block["attributes"]["tags"]["labels"])
    values = np.asarray(block["attributes"]["tags"]["values"]) > 0
    P = np.asarray(mesh["V"])[np.asarray(block["connectivity"])]
    facets, cells = {}, set()
    for tag in tags:
        for corners in P[values[:, labels.index(tag)] & (values.sum(axis=1) == 1)]:
            points = [tuple(c) for c in corners.tolist()]
            cells.add(frozenset(points))
            for facet in itertools.combinations(points, len(points) - 1):
                facets[frozenset(facet)] = tag
    return facets, cells


@pytest.mark.parametrize("case", ["four_bodies", "two_bodies_2d", "two_bodies_3d"])
def test_split_fill_gives_a_cell_on_a_region_that_region(case):
    mesh, tags, distance = {
        "four_bodies": (_four_bodies(), TAGS, 2.5),
        "two_bodies_2d": (_two_bodies_3_apart(2), ["a", "b"], 3.5),
        "two_bodies_3d": (_two_bodies_3_apart(3), ["a", "b"], 3.5),
    }[case]
    out = _seal(mesh, mode="split_fill", tags=tags, distance=distance)
    facets, own = _region_facets(mesh, tags)
    seen = 0
    for tag in tags:
        for corners in _tagged(out, tag):
            points = [tuple(c) for c in corners.tolist()]
            if frozenset(points) in own:
                continue  # a region's own cell, not one the fill labelled
            for facet in itertools.combinations(points, len(points) - 1):
                if frozenset(facet) in facets:
                    assert facets[frozenset(facet)] == tag
                    seen += 1
    assert seen > 0


def test_split_fill_has_no_limit_of_three_regions():
    mesh = _four_bodies()
    out = _seal(mesh, mode="split_fill", tags=TAGS)
    assert sum(_area(out, tag) for tag in TAGS) == pytest.approx(
        sum(_area(mesh, tag) for tag in TAGS) + 4)
    for tag in TAGS:
        assert _components_and_euler(_cells(out, tag)) == (1, 1)


def test_voronoi_is_still_the_default():
    mesh = _slot()
    default = _seal(mesh, tags=["a", "b"])
    named = _seal(mesh, mode="voronoi", tags=["a", "b"])
    assert np.array_equal(default["V"], named["V"])
    assert np.array_equal(
        default["cells"]["triangle"]["connectivity"], named["cells"]["triangle"]["connectivity"])
