"""Tier 1 — the interface artifacts the C++ engine writes for polyfem, on a
synthetic 3D mesh, plus a self-check of the gap metric the separation test
relies on. `inputs_only` makes the engine stop after writing them."""
import numpy as np
import h5py
import pytest

from conftest import needs_polyfem_ops, run_cpp
from geo import min_separation_3d

SELS = [{"region": "tag_0", "filter": "ambient", "id": 1},
        {"region": "tag_1", "filter": "ambient", "id": 2}]


def _obj(path):
    """(vertices, faces) of an OBJ, faces as 0-based vertex indices."""
    verts, faces = [], []
    for line in open(path):
        p = line.split()
        if p and p[0] == "v":
            verts.append([float(x) for x in p[1:4]])
        elif p and p[0] == "f":
            faces.append([int(x.split("/")[0]) - 1 for x in p[1:4]])
    return np.array(verts), np.array(faces)


def _body_ids(path):
    return [[int(x) for x in ln.split()] for ln in open(path)]


@pytest.fixture()
def artifacts(boxes3d, tmp_path):
    # Scale 1: the collision proxy is then in mesh units, where the box
    # centers below are.
    run_cpp(boxes3d, "minimum_separation", tmp_path, collision_pairs=[SELS],
            sep=1.5, normalize_penalties=False, scale=1.0)
    return tmp_path / "sep_input"


@needs_polyfem_ops
def test_skin_face_counts(artifacts):
    # Each body is a 1x2x2 box fully interior to the ambient grid; its skin
    # is 16 unit quads = 32 triangles under the Freudenthal split.
    ids = _body_ids(artifacts / "collision_body_ids.txt")
    assert ids.count([1]) == 32
    assert ids.count([2]) == 32


def test_gap_metric_on_input(boxes3d):
    # Bodies sit at x in [1,2] and [3,4]: the true surface gap is exactly 1.
    assert min_separation_3d(boxes3d, {"region": "tag_0"},
                             {"region": "tag_1"}) == pytest.approx(1.0)


@needs_polyfem_ops
def test_fitting_constraint_is_positive_diagonal(artifacts):
    # Diagonal (rows == cols) with strictly positive per-node mass weights,
    # zero RHS -> penalizes any displacement of the interface nodes.
    with h5py.File(artifacts / "interface_constraint.hdf5") as f:
        n = f["local2global"].shape[0]
        rows = f["A_triplets/rows"][()]
        cols = f["A_triplets/cols"][()]
        vals = f["A_triplets/values"][()]
        assert np.array_equal(rows, cols)
        assert (vals > 0).all()
        assert list(f["A_triplets/shape"][()]) == [n, n]
        assert not f["b"][()].any()


@needs_polyfem_ops
def test_laplacian_rows_sum_to_zero(artifacts):
    import scipy.sparse as sp
    with h5py.File(artifacts / "interface_constraint_laplacian.hdf5") as f:
        shape = f["A_triplets/shape"][()]
        L = sp.coo_matrix(
            (f["A_triplets/values"][()],
             (f["A_triplets/rows"][()], f["A_triplets/cols"][()])),
            shape=tuple(shape)).tocsr()
    assert np.allclose(np.asarray(L.sum(axis=1)).ravel(), 0.0, atol=1e-10)


@needs_polyfem_ops
def test_collision_proxy_outward_and_body_ids(artifacts):
    V, F = _obj(artifacts / "interface_collision.obj")
    ids = _body_ids(artifacts / "collision_body_ids.txt")
    assert len(ids) == len(F) == 64
    assert {i for row in ids for i in row} == {1, 2}

    # Normals must point away from each body's interior (bodies are convex).
    centers = {1: np.array([1.5, 2.0, 2.0]), 2: np.array([3.5, 2.0, 2.0])}
    for f, row in zip(F, ids):
        a, b, c = V[f]
        n = np.cross(b - a, c - a)
        assert np.dot(n, (a + b + c) / 3.0 - centers[row[0]]) > 0


def _smoothing_faces(mesh, out_dir, interfaces):
    """The collision-proxy faces the smoothing operation writes for
    `interfaces`, each as the set of its vertex positions."""
    run_cpp(mesh, "laplacian_smoothing", out_dir, interfaces=interfaces)
    V, F = _obj(out_dir / "smooth_input" / "interface_collision.obj")
    return {frozenset(map(tuple, V[f])) for f in F}


@needs_polyfem_ops
def test_union_region_selects_both_skins(boxes3d, tmp_path):
    # A union region: boundary of (tag_0 | tag_1) facing ambient = both skins.
    # Paired with itself it dedupes to one side, hence one body.
    union = {"region": "tag_0 | tag_1", "filter": "ambient"}
    run_cpp(boxes3d, "minimum_separation", tmp_path,
            collision_pairs=[[union, union]], sep=1.5)
    ids = _body_ids(tmp_path / "sep_input" / "collision_body_ids.txt")
    assert len(ids) == 64
    assert all(t == [1] for t in ids)


@needs_polyfem_ops
def test_whole_boundary_and_filter_agree_on_interior_bodies(boxes3d, tmp_path):
    # Fully interior body: whole boundary == ambient-filtered boundary.
    whole = _smoothing_faces(boxes3d, tmp_path / "whole", ["tag_0"])
    filtered = _smoothing_faces(boxes3d, tmp_path / "filtered",
                                [{"region": "tag_0", "filter": "ambient"}])
    assert len(whole) == 32
    assert whole == filtered


@needs_polyfem_ops
def test_underscore_rejected(boxes3d, tmp_path):
    with pytest.raises(RuntimeError, match="ambient"):
        run_cpp(boxes3d, "laplacian_smoothing", tmp_path,
                interfaces=[{"region": "tag_0", "filter": "_"}])
