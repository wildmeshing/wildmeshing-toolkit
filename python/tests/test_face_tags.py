"""Face tags: surfaces kept as tagged faces next to tagged volumes (the shells pipeline).

Inputs, one per mesh, tagged by input name:
- red:    a cube [0,1]^3, a volume;
- green:  a closed box [3,4]x[0,1]x[0,1] that stays a surface;
- yellow: an open square at z = 0.5, [1.5,2.3]x[0.1,0.9].

Step 1, reader:     a triangle block with tags passes through a simwild operation.
Step 2, generation: input_type decides tet tags (volume) or face tags (surface).
Step 3, ops:        tagged faces stay within eps of their input, both ways.

simwild's guarantee is within eps of the inputs, not exact, so the checks are two-sided
distances against eps (eps_rel times the bbox diagonal).
Step 4, offset:     offset_selection names a face tag and grows a shell on both sides.
"""
from itertools import permutations

import numpy as np
import pytest

import wildmeshing as wm

BOX_F = [(0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (3, 7, 6), (3, 6, 2), (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5)]
SQUARE = (1.5, 2.3, 0.1, 0.9, 0.5)  # x0, x1, y0, y1, z
GREEN = ((3, 0, 0), (4, 1, 1))
EPS_REL = 1e-2
EPS = EPS_REL * np.linalg.norm([4, 1, 1])  # inputs' bbox [0,4]x[0,1]x[0,1]


def tri_mesh(V, F):
    return {"V": np.asarray(V, float), "cells": {"triangle": {"connectivity": np.asarray(F, np.int64)}}}


def box(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    return tri_mesh([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                     (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], BOX_F)


def square(n=4):
    x0, x1, y0, y1, z = SQUARE
    V = [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * j / n, z) for j in range(n + 1) for i in range(n + 1)]
    k = lambda i, j: j * (n + 1) + i
    F = [f for j in range(n) for i in range(n)
         for f in ((k(i, j), k(i + 1, j), k(i + 1, j + 1)), (k(i, j), k(i + 1, j + 1), k(i, j + 1)))]
    return tri_mesh(V, F)


INPUTS = {"cube": box((0, 0, 0), (1, 1, 1)), "box": box(*GREEN), "square": square()}
GEN = {"input": ["cube", "box", "square"], "input_names": ["red", "green", "yellow"],
       "input_type": ["volume", "surface", "surface"], "skip_simplify": True, "eps_rel": EPS_REL}


def generate(**extra):
    return wm.simwild({**GEN, **extra}, INPUTS)[0]


# ---------------------------------------------------------------- geometry


def rows(m, cell, label):
    """Connectivity of the `cell` rows tagged `label` (empty if none)."""
    c = m["cells"].get(cell)
    if c is None or "tags" not in c.get("attributes", {}):
        return np.zeros((0, 3 if cell == "triangle" else 4), np.int64)
    t = c["attributes"]["tags"]
    if label not in t["labels"]:
        return np.zeros((0, c["connectivity"].shape[1]), np.int64)
    return c["connectivity"][np.asarray(t["values"])[:, list(t["labels"]).index(label)] == 1]


def area(V, F):
    P = V[F]
    return 0.5 * np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1).sum()


def volume(V, T):
    P = V[T]
    return np.abs(np.einsum("ij,ij->i", np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), P[:, 3] - P[:, 0])).sum() / 6


def to_rect(P, rect=SQUARE):
    x0, x1, y0, y1, z = rect
    dx = np.maximum(np.maximum(x0 - P[..., 0], 0), P[..., 0] - x1)
    dy = np.maximum(np.maximum(y0 - P[..., 1], 0), P[..., 1] - y1)
    return np.sqrt(dx ** 2 + dy ** 2 + (P[..., 2] - z) ** 2)


def to_box_surface(P, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    outside = np.linalg.norm(np.maximum(np.maximum(lo - P, 0), P - hi), axis=-1)
    inside = np.min(np.minimum(P - lo, hi - P), axis=-1)
    return np.where(outside > 0, outside, np.abs(inside))


def to_faces(p, V, F):
    """Exact distance from point p to the triangles V[F] (closest point, Ericson 5.1.5)."""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    ab, ac, ap, bp, cp = b - a, c - a, p - a, p - b, p - c
    dot = lambda u, v: np.einsum("ij,ij->i", u, v)
    d1, d2, d3, d4, d5, d6 = dot(ab, ap), dot(ac, ap), dot(ab, bp), dot(ac, bp), dot(ab, cp), dot(ac, cp)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    with np.errstate(divide="ignore", invalid="ignore"):
        s = 1 / (va + vb + vc)
        q = np.select(
            [(d1 <= 0) & (d2 <= 0), (d3 >= 0) & (d4 <= d3), (vc <= 0) & (d1 >= 0) & (d3 <= 0),
             (d6 >= 0) & (d5 <= d6), (vb <= 0) & (d2 >= 0) & (d6 <= 0),
             (va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0)],
            [a.T, b.T, (a + (d1 / (d1 - d3))[:, None] * ab).T, c.T, (a + (d2 / (d2 - d6))[:, None] * ac).T,
             (b + ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[:, None] * (c - b)).T],
            (a + (vb * s)[:, None] * ab + (vc * s)[:, None] * ac).T).T
    return np.linalg.norm(q - p, axis=1).min()


def rect_samples(rect=SQUARE, n=10):
    x0, x1, y0, y1, z = rect
    return [np.array([x, y, z]) for x in np.linspace(x0, x1, n) for y in np.linspace(y0, y1, n)]


def box_samples(lo, hi, n=6):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    g = np.linspace(0, 1, n)
    out = []
    for axis in range(3):
        for side in (lo[axis], hi[axis]):
            u, v = [k for k in range(3) if k != axis]
            for s in g:
                for t in g:
                    p = np.empty(3)
                    p[axis], p[u], p[v] = side, lo[u] + s * (hi[u] - lo[u]), lo[v] + t * (hi[v] - lo[v])
                    out.append(p)
    return out


def boundary(T):
    """The faces of the tets T that only one of them has."""
    F = np.vstack([T[:, [0, 1, 2]], T[:, [0, 1, 3]], T[:, [0, 2, 3]], T[:, [1, 2, 3]]])
    key, count = np.unique(np.sort(F, 1), axis=0, return_counts=True)
    return key[count == 1]


def assert_hausdorff(V, F, dist_to_input, samples, eps):
    """The faces V[F] and their input are within eps of each other, both ways."""
    assert len(F), "no faces"
    assert dist_to_input(V[F]).max() <= eps
    assert max(to_faces(p, V, F) for p in samples) <= eps


# ---------------------------------------------------------------- step 2: generation
# simwild guarantees its surfaces within eps of the inputs, not exactly on them: even with
# max_iterations 0, 14 of 692 red tets reach outside the cube (measured 2026-10-27).

CUBE = ((0, 0, 0), (1, 1, 1))


def check_volume(m, label, lo_hi):
    T = rows(m, "tetrahedron", label)
    assert len(T), f"no tet tagged {label}"
    assert_hausdorff(m["V"], boundary(T), lambda P: to_box_surface(P, *lo_hi), box_samples(*lo_hi), EPS)


def test_generation_default_makes_every_input_a_volume():
    """Without input_type, every closed input is a volume, as before."""
    options = {k: v for k, v in GEN.items() if k != "input_type"}
    m = wm.simwild(options, INPUTS)[0]["mesh"]
    check_volume(m, "red", CUBE)
    check_volume(m, "green", GREEN)
    assert len(rows(m, "tetrahedron", "yellow")) == 0


def test_generation_refuses_input_type_of_the_wrong_length():
    with pytest.raises(Exception, match="one entry per input"):
        wm.simwild({**GEN, "input_type": ["volume", "surface"]}, INPUTS)


def test_generation_refuses_an_unknown_input_type():
    """Only volume and surface exist; curve is not built yet."""
    with pytest.raises(Exception, match="input_type"):
        wm.simwild({**GEN, "input_type": ["volume", "curve", "surface"]}, INPUTS)


@pytest.mark.parametrize("iterations", [0, 80])
def test_generation_keeps_surface_inputs_as_tagged_faces(iterations):
    """red is a volume; green and yellow are faces within eps of their inputs. No tet carries a
    surface's tag, and no face carries the volume's tag."""
    m = generate(max_iterations=iterations)["mesh"]
    check_volume(m, "red", CUBE)
    assert len(rows(m, "tetrahedron", "green")) == 0 and len(rows(m, "tetrahedron", "yellow")) == 0
    assert len(rows(m, "triangle", "red")) == 0
    assert_hausdorff(m["V"], rows(m, "triangle", "green"), lambda P: to_box_surface(P, *GREEN),
                     box_samples(*GREEN), EPS)
    assert_hausdorff(m["V"], rows(m, "triangle", "yellow"), to_rect, rect_samples(), EPS)


def normals(V, F):
    V, F = np.asarray(V), np.asarray(F)
    return np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])


def flipped(mesh):
    return tri_mesh(mesh["V"], np.asarray(mesh["cells"]["triangle"]["connectivity"])[:, [0, 2, 1]])


@pytest.mark.parametrize("iterations", [0, 80])
@pytest.mark.parametrize("flip", [False, True])
def test_generation_orients_tagged_faces_like_their_inputs(iterations, flip):
    """Each tagged face points as its input does: yellow along the square's normal, green out of
    or into the box as its triangles do, and the other way with both inputs flipped. The
    remesher's own vertex order carries no orientation."""
    inputs = dict(INPUTS)
    if flip:
        inputs["box"], inputs["square"] = flipped(inputs["box"]), flipped(inputs["square"])
    m = wm.simwild({**GEN, "max_iterations": iterations}, inputs)[0]["mesh"]
    V = np.asarray(m["V"])

    sq = inputs["square"]
    want = set(np.sign(normals(sq["V"], sq["cells"]["triangle"]["connectivity"])[:, 2]))
    F = rows(m, "triangle", "yellow")
    assert len(want) == 1 and len(F) > 0
    assert set(np.sign(normals(V, F)[:, 2])) == want

    center = (np.array(GREEN[0]) + np.array(GREEN[1])) / 2
    BV = np.asarray(inputs["box"]["V"])
    BF = np.asarray(inputs["box"]["cells"]["triangle"]["connectivity"])
    want = set(np.sign(np.einsum("ij,ij->i", normals(BV, BF), BV[BF].mean(1) - center)))
    F = rows(m, "triangle", "green")
    assert len(want) == 1 and len(F) > 0
    assert set(np.sign(np.einsum("ij,ij->i", normals(V, F), V[F].mean(1) - center))) == want


# ---------------------------------------------------------------- steps 1, 3, 4: a tagged tet mesh
# Built by hand, so these do not depend on generation: [0,2]^3 in 8^3 cube cells of 6 tets,
# red the corner cell [0,0.25]^3, and two open sheets in one triangle block: yellow on z = 1
# and blue on z = 0.25, both inside [0.5,1.5]^2. With two labels, today's reader makes the
# whole block one unnamed sheet, so selecting yellow alone needs real face tags.

SHEET = (0.5, 1.5, 0.5, 1.5, 1.0)
BLUE = (0.5, 1.5, 0.5, 1.5, 0.25)
RED = ((0, 0, 0), (0.25, 0.25, 0.25))
EPS_K = EPS_REL * np.linalg.norm([2, 2, 2])


def kuhn_grid(n=8, h=0.25):
    """[0, n h]^3, each cube cell split into 6 tets around its diagonal."""
    vid = lambda i, j, k: (k * (n + 1) + j) * (n + 1) + i
    V = [(i * h, j * h, k * h) for k in range(n + 1) for j in range(n + 1) for i in range(n + 1)]
    T, cells = [], []
    for k in range(n):
        for j in range(n):
            for i in range(n):
                for perm in permutations(range(3)):
                    c, t = [i, j, k], [vid(i, j, k)]
                    for s in perm:
                        c[s] += 1
                        t.append(vid(*c))
                    T.append(t)
                    cells.append((i, j, k))
    V, T = np.asarray(V, float), np.asarray(T, np.int64)
    P = V[T]  # orient positively
    neg = np.einsum("ij,ij->i", np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), P[:, 3] - P[:, 0]) < 0
    T[neg] = T[neg][:, [1, 0, 2, 3]]
    return V, T, cells


def tagged_grid():
    V, T, cells = kuhn_grid()
    red = np.array([c == (0, 0, 0) for c in cells], np.uint8)
    faces = np.unique(np.sort(np.vstack([T[:, [0, 1, 2]], T[:, [0, 1, 3]], T[:, [0, 2, 3]], T[:, [1, 2, 3]]]), 1), axis=0)
    yellow = faces[to_rect(V[faces], SHEET).max(1) == 0]
    blue = faces[to_rect(V[faces], BLUE).max(1) == 0]
    member = np.zeros((len(yellow) + len(blue), 2), np.uint8)
    member[:len(yellow), 0] = member[len(yellow):, 1] = 1
    return {"V": V, "cells": {
        "tetrahedron": {"connectivity": T, "attributes": {
            "tags": {"values": np.stack([red, 1 - red], 1), "labels": ["red", "ambient"]}}},
        "triangle": {"connectivity": np.vstack([yellow, blue]), "attributes": {
            "tags": {"values": member, "labels": ["yellow", "blue"]}}}}}


def test_tagged_faces_pass_through_an_operation():
    """Step 1: an operation that moves nothing returns the same yellow and blue faces."""
    m = tagged_grid()
    assert len(rows(m, "triangle", "yellow")) == len(rows(m, "triangle", "blue")) == 32  # 4x4 squares, 2 triangles each
    r = wm.simwild({"operation": "remeshing", "input": ["m"], "max_iterations": 0}, {"m": m})[0]["mesh"]
    key = lambda M, F: sorted(tuple(sorted(map(tuple, M["V"][f]))) for f in F)
    for label in ["yellow", "blue"]:
        assert key(r, rows(r, "triangle", label)) == key(m, rows(m, "triangle", label))


def test_remeshing_keeps_tagged_faces_within_eps():
    """Step 3: remeshing keeps the sheet and the red volume within eps of where they were."""
    r = wm.simwild({"operation": "remeshing", "input": ["m"], "eps_rel": EPS_REL}, {"m": tagged_grid()})[0]["mesh"]
    for label, rect in [("yellow", SHEET), ("blue", BLUE)]:
        assert_hausdorff(r["V"], rows(r, "triangle", label), lambda P: to_rect(P, rect),
                         rect_samples(rect), EPS_K)
    T = rows(r, "tetrahedron", "red")
    assert_hausdorff(r["V"], boundary(T), lambda P: to_box_surface(P, *RED), box_samples(*RED), EPS_K)


def test_offset_grows_a_shell_around_a_tagged_sheet():
    """Step 4: offset_selection names the yellow faces. Every yellow face ends up between two
    shell tets, one on each side, and every shell tet touches yellow, so blue is not offset."""
    m = tagged_grid()
    r = wm.topological_offset({"input": "m", "offset_selection": "yellow", "offset_output_tags": ["shell"],
                               "target_distance": 0.1, "optimize_offset": False}, {"m": m})[0]["mesh"]
    V, shell = r["V"], rows(r, "tetrahedron", "shell")
    assert len(shell), "no shell"
    assert (to_rect(V[shell], SHEET).min(1) < 1e-9).all()
    count = {}
    for t in shell:
        for f in (t[[0, 1, 2]], t[[0, 1, 3]], t[[0, 2, 3]], t[[1, 2, 3]]):
            count[tuple(sorted(f))] = count.get(tuple(sorted(f)), 0) + 1
    yellow = rows(r, "triangle", "yellow")
    assert len(yellow) and all(count.get(tuple(sorted(f)), 0) == 2 for f in yellow)
    assert len(rows(r, "triangle", "blue")) == 32
    assert len(rows(r, "tetrahedron", "red")) == len(rows(m, "tetrahedron", "red"))
