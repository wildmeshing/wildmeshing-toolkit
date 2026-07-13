"""Tier 1 — make_interface_constraint artifacts on a synthetic 3D mesh,
plus a self-check of the gap metric the separation test relies on."""
import numpy as np
import h5py
import pytest

from simwild.polyfem_ops import constraints as mic

from geo import min_separation_3d

SELS = [{"region": "tag_0", "filter": "ambient", "id": 1},
        {"region": "tag_1", "filter": "ambient", "id": 2}]


@pytest.fixture()
def artifacts(boxes3d, tmp_path):
    out = tmp_path / "constraints"
    out.mkdir()
    mic.make_interface_constraint(
        mesh_path=str(boxes3d), selections=SELS, out_dir=str(out),
        normalize=False, scale=1.0)
    return out


def test_skin_face_counts(boxes3d):
    # Each body is a 1x2x2 box fully interior to the ambient grid; its skin
    # is 16 unit quads = 32 triangles under the Freudenthal split.
    (_, _, _, _, dim, faces, _, _, face_tags) = mic.load_mesh(
        str(boxes3d), selections=SELS)
    assert dim == 3
    ids = np.array([t[0] for t in face_tags])
    assert (ids == 1).sum() == 32
    assert (ids == 2).sum() == 32


def test_gap_metric_on_input(boxes3d):
    # Bodies sit at x in [1,2] and [3,4]: the true surface gap is exactly 1.
    assert min_separation_3d(boxes3d, {"region": "tag_0"},
                             {"region": "tag_1"}) == pytest.approx(1.0)


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


def test_laplacian_rows_sum_to_zero(artifacts):
    import scipy.sparse as sp
    with h5py.File(artifacts / "interface_constraint_laplacian.hdf5") as f:
        shape = f["A_triplets/shape"][()]
        L = sp.coo_matrix(
            (f["A_triplets/values"][()],
             (f["A_triplets/rows"][()], f["A_triplets/cols"][()])),
            shape=tuple(shape)).tocsr()
    assert np.allclose(np.asarray(L.sum(axis=1)).ravel(), 0.0, atol=1e-10)


def test_collision_proxy_outward_and_body_ids(artifacts, boxes3d):
    verts, faces = [], []
    for line in open(artifacts / "interface_collision.obj"):
        p = line.split()
        if p and p[0] == "v":
            verts.append([float(x) for x in p[1:4]])
        elif p and p[0] == "f":
            faces.append([int(x.split("/")[0]) - 1 for x in p[1:4]])
    V, F = np.array(verts), np.array(faces)
    ids = [[int(x) for x in ln.split()]
           for ln in open(artifacts / "collision_body_ids.txt")]
    assert len(ids) == len(F) == 64
    assert {i for row in ids for i in row} == {1, 2}

    # Normals must point away from each body's interior (bodies are convex).
    centers = {1: np.array([1.5, 2.0, 2.0]), 2: np.array([3.5, 2.0, 2.0])}
    for f, row in zip(F, ids):
        a, b, c = V[f]
        n = np.cross(b - a, c - a)
        assert np.dot(n, (a + b + c) / 3.0 - centers[row[0]]) > 0


def test_union_region_selects_both_skins(boxes3d):
    # A union region: boundary of (tag_0 | tag_1) facing ambient = both skins.
    (_, _, _, _, _, faces, _, _, tags) = mic.load_mesh(
        boxes3d, selections=[{"region": "tag_0 | tag_1", "filter": "ambient"}])
    assert len(faces) == 64
    assert all(t == [1] for t in tags)


def test_whole_boundary_and_filter_agree_on_interior_bodies(boxes3d):
    # Fully interior body: whole boundary == ambient-filtered boundary.
    whole = mic.load_mesh(boxes3d, selections=["tag_0"])[5]
    filtered = mic.load_mesh(
        boxes3d, selections=[{"region": "tag_0", "filter": "ambient"}])[5]
    assert sorted(map(sorted, whole)) == sorted(map(sorted, filtered))


def test_underscore_rejected(boxes3d):
    with pytest.raises(ValueError, match="ambient"):
        mic.load_mesh(boxes3d, selections=[{"region": "tag_0", "filter": "_"}])
