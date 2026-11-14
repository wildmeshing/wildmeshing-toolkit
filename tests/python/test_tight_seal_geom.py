"""tight_seal_geom: what a fill does.

The region given by tight_seal_geom_input_selection is closed; each part of the join region that
borders enough colors (tight_seal_geom_fill_selections) is split along the colors nearest its
cells' corners, a cell sharing a facet with a colored cell labelling that facet's corners with
it, and each resulting cell gets its color's tags (tight_seal_geom_fill_tags) added to its own,
with no topology test. The operation throws if the join region touches the mesh boundary, even at
a vertex, or if a cell beside it satisfies several fill selections.

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


def _seal(mesh, **options):
    opts = {"operation": "tight_seal_geom", "input": ["m"], "tight_seal_geom_distance": 2.5,
            "tight_seal_geom_sanity": True}
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


def _colors(tags):
    """Close the union of `tags`, each tag a color that adds itself."""
    return {"input_selection": " | ".join(tags), "fill_selections": list(tags),
            "fill_tags": [[t] for t in tags]}


def test_needs_an_input_selection():
    with pytest.raises(Exception, match="tight_seal_geom_input_selection"):
        _seal(_four_bodies(), fill_selections=TAGS, fill_tags=[[t] for t in TAGS])


def test_needs_two_fill_selections():
    with pytest.raises(Exception, match="two or more"):
        _seal(_four_bodies(), input_selection="a | b", fill_selections=["a"], fill_tags=[["a"]])


def test_needs_one_fill_tag_set_per_fill_selection():
    with pytest.raises(Exception, match="one tag set per"):
        _seal(_four_bodies(), input_selection="a | b", fill_selections=["a", "b"],
              fill_tags=[["a"]])


def test_fill_tags_cannot_hold_ambient():
    with pytest.raises(Exception, match="ambient"):
        _seal(_four_bodies(), input_selection="a | b", fill_selections=["a", "b"],
              fill_tags=[["a"], ["ambient"]])


def test_ambiguous_fill_selections_throw():
    # the cells of b beside the hole satisfy both "a | b" and "b"
    with pytest.raises(Exception, match="ambiguous"):
        _seal(_four_bodies(), input_selection="a | b | c | d",
              fill_selections=["a | b", "b", "c", "d"], fill_tags=[["a"], ["b"], ["c"], ["d"]])


def test_min_regions_below_two_refused():
    with pytest.raises(Exception):
        _seal(_four_bodies(), **_colors(TAGS), min_regions=1)


@pytest.mark.parametrize("min_regions", [2, 4])
def test_fills_a_part_of_any_number_of_colors(min_regions):
    mesh = _four_bodies()
    out = _seal(mesh, **_colors(TAGS), min_regions=min_regions)
    # the whole hole goes, each side of it to the bar beside it
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(_area(mesh, tag) + 1)
        assert _components_and_euler(_cells(out, tag)) == (1, 1)


def test_max_preprocessing_rounds_below_zero_refused():
    with pytest.raises(Exception):
        _seal(_four_bodies(), **_colors(TAGS), max_preprocessing_rounds=-1)


@pytest.mark.parametrize("rounds", [0, 1, 10])
def test_max_preprocessing_rounds_taken(rounds):
    # the hole's sides are each nearest one bar, so the preprocessing has nothing to split here
    mesh = _four_bodies()
    out = _seal(mesh, **_colors(TAGS), max_preprocessing_rounds=rounds)
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(_area(mesh, tag) + 1)


def test_min_regions_above_what_a_part_borders_fills_nothing():
    mesh = _four_bodies()
    out = _seal(mesh, **_colors(TAGS), min_regions=5)
    for tag in TAGS:
        assert _area(out, tag) == pytest.approx(_area(mesh, tag))


@pytest.mark.parametrize("mesh", [_gap_to_the_boundary, _diamond_on_the_boundary])
def test_join_region_on_the_mesh_boundary_throws(mesh):
    with pytest.raises(Exception, match="touches the mesh boundary"):
        _seal(mesh(), **_colors(["a", "b"]), distance=4.0)


def test_part_left_alone_on_the_mesh_boundary_throws_too():
    # a body one cell from the wall: the strip between borders one color and would not be filled
    def tag_of(x, y):
        if 2 < y < 6:
            return "a" if 1 < x < 4 else ("b" if 8 < x < 14 else None)
        return None
    with pytest.raises(Exception, match=r"touches the mesh boundary.* 0 of them in parts to fill"):
        _seal(_grid_mesh(16, 8, tag_of), **_colors(["a", "b"]))


def test_fill_tags_are_added_to_a_cells_own():
    # the band of c across the gap is outside the closed region and no color: its cells are
    # filled like ambient ones and keep c
    mesh = _gap_with_another_tag()
    out = _seal(mesh, **_colors(["a", "b"]))
    assert _area(out, "c") == pytest.approx(_area(mesh, "c"))
    c = {frozenset(map(tuple, corners.tolist())) for corners in _tagged(out, "c")}
    for tag in ["a", "b"]:
        assert any(frozenset(map(tuple, corners.tolist())) in c for corners in _tagged(out, tag))


def test_fill_tags_may_be_new_and_several():
    mesh = _two_bodies_3_apart(2)
    out = _seal(mesh, input_selection="a | b", fill_selections=["a", "b"],
                fill_tags=[["a", "seal"], ["b", "seal"]], distance=3.5)
    gained = _area(out, "a") - _area(mesh, "a") + _area(out, "b") - _area(mesh, "b")
    assert gained > 0
    assert _area(out, "seal") == pytest.approx(gained)


def test_a_color_outside_the_closed_region():
    # ambient beside the join region is a color too: the fill nearest it adds no tag
    mesh = _two_bodies_3_apart(2)
    two = _seal(mesh, **_colors(["a", "b"]), distance=3.5)
    three = _seal(mesh, input_selection="a | b", fill_selections=["a", "b", "_"],
                  fill_tags=[["a"], ["b"], []], distance=3.5)
    def gained(out):
        return _area(out, "a") - _area(mesh, "a") + _area(out, "b") - _area(mesh, "b")
    assert 0 < gained(three) < gained(two)


@pytest.mark.parametrize("dim", [2, 3])
def test_fill_follows_the_bisector(dim):
    mesh = _two_bodies_3_apart(dim)
    out = _seal(mesh, **_colors(["a", "b"]), distance=3.5)
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
def test_a_cell_on_a_color_takes_that_color(case):
    mesh, tags, distance = {
        "four_bodies": (_four_bodies(), TAGS, 2.5),
        "two_bodies_2d": (_two_bodies_3_apart(2), ["a", "b"], 3.5),
        "two_bodies_3d": (_two_bodies_3_apart(3), ["a", "b"], 3.5),
    }[case]
    out = _seal(mesh, **_colors(tags), distance=distance)
    facets, own = _region_facets(mesh, tags)
    seen = 0
    for tag in tags:
        for corners in _tagged(out, tag):
            points = [tuple(c) for c in corners.tolist()]
            if frozenset(points) in own:
                continue  # a color's own cell, not one the fill labelled
            for facet in itertools.combinations(points, len(points) - 1):
                if frozenset(facet) in facets:
                    assert facets[frozenset(facet)] == tag
                    seen += 1
    assert seen > 0


# tight_seal_geom_EXPERIMENTAL_material_interface_fill: each part is cut by the exact minimization
# diagram of the distances to its colors, interpolated linearly in each tet, and each piece goes to
# its nearest color.

def _four_bars_3d():
    """_four_bodies' bars, between z = 2 and z = 4: a above the hole (5 < y < 6), b right of it
    (5 < x < 6), c below (2 < y < 3), d left (2 < x < 3), around the hole 3 < x, y < 5."""
    def tag_of(x, y, z):
        if not 2 < z < 4:
            return None
        if 3 < x < 5 and 3 < y < 5 or not (2 < x < 6 and 2 < y < 6):
            return None
        if y > 5 and x < 5:
            return "a"
        if x > 5 and y > 3:
            return "b"
        return "c" if y < 3 else "d"
    return _grid_mesh_3d(8, 8, 6, tag_of)


def test_material_interface_fill_follows_the_bisector():
    mesh = _two_bodies_3_apart(3)
    out = _seal(mesh, **_colors(["a", "b"]), distance=3.5,
                EXPERIMENTAL_material_interface_fill=True)
    a = _tagged(out, "a")[:, :, 0]
    b = _tagged(out, "b")[:, :, 0]
    filled_a = a[a.max(axis=1) > 4 + 1e-9]
    filled_b = b[b.min(axis=1) < 7 - 1e-9]
    assert len(filled_a) > 0 and len(filled_b) > 0
    # the distances are linear in x across the gap: the diagram is the bisector, exactly
    assert filled_a.max() <= 5.5 + 1e-9
    assert filled_b.min() >= 5.5 - 1e-9


def test_material_interface_fill_takes_each_cell_to_its_nearest_color():
    mesh = _four_bars_3d()
    out = _seal(mesh, **_colors(TAGS), EXPERIMENTAL_material_interface_fill=True)
    # in the hole, the distances to the bars are 5 - y, 5 - x, y - 3 and x - 3: the diagram is
    # its two diagonals, and the inside of each piece is strictly nearest its bar
    for tag in TAGS:
        centroids = _tagged(out, tag).mean(axis=1)
        x, y, z = centroids.T
        inside = (3 < x) & (x < 5) & (3 < y) & (y < 5) & (2 < z) & (z < 4)
        assert inside.sum() > 0
        d = {"a": 5 - y, "b": 5 - x, "c": y - 3, "d": x - 3}
        for other in TAGS:
            if other != tag:
                assert np.all(d[tag][inside] < d[other][inside] - 1e-9)


def test_material_interface_fill_is_3d_only():
    with pytest.raises(Exception, match="3D only"):
        _seal(_four_bodies(), **_colors(TAGS), EXPERIMENTAL_material_interface_fill=True)
