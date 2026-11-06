"""wildmeshing.viewer registers every layer of a chain without a window (polyscope's mock
backend): volume tags on tets and on a prism, face and edge tags, and a labelled envelope."""
import numpy as np
import pytest

polyscope = pytest.importorskip("polyscope")
from wildmeshing import viewer  # noqa: E402

# The unit cube in six tets around its diagonal, two regions; a prism beside it.
V = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1], [0, 1, 1],
              [1, 1, 1], [2, 0, 0], [3, 0, 0], [2, 1, 0], [2, 0, 1], [3, 0, 1], [2, 1, 1]], float)
TETS = np.array([[0, 1, 3, 7], [0, 1, 7, 5], [0, 2, 7, 3], [0, 2, 6, 7], [0, 4, 5, 7],
                 [0, 4, 7, 6]], np.int64)


def tags(values, labels):
    return {"tags": {"values": np.asarray(values, np.uint8), "labels": labels}}


def stage():
    region = np.array([[1, 0], [1, 0], [1, 0], [0, 1], [0, 1], [0, 1]])
    mesh = {"V": V, "cells": {
        "tetrahedron": {"connectivity": TETS, "attributes": {
            **tags(region, ["a", "b"]), "quality": {"values": np.arange(6, dtype=float)}}},
        "prism": {"connectivity": np.array([[8, 9, 10, 11, 12, 13]], np.int64),
                  "attributes": tags([[1, 0]], ["a", "b"])},
        "triangle": {"connectivity": np.array([[0, 1, 7], [0, 3, 7]], np.int64),
                     "attributes": tags([[1], [1]], ["sheet"])},
        "edge": {"connectivity": np.array([[0, 7]], np.int64),
                 "attributes": tags([[1]], ["feature_0"])}}}
    envelope = {"V": V, "cells": {
        "triangle": {"connectivity": np.array([[0, 1, 7], [0, 3, 7]], np.int64),
                     "attributes": tags([[1, 0], [0, 1]], ["sheet", "a"])}}}
    return {"mesh": mesh, "envelope": envelope}


def test_view_registers_every_layer():
    viewer.view([("first", stage()), ("second", {"mesh": stage()["mesh"]})], screenshots="mock")
    for name in ["first volume a tets", "first volume a prisms", "first volume b tets",
                 "first face sheet", "first edge feature_0", "first face sheet envelope triangles",
                 "first volume a envelope triangles", "second volume a tets"]:
        assert polyscope.has_surface_mesh(name) or polyscope.has_volume_mesh(name) \
            or polyscope.has_curve_network(name), name
    assert not polyscope.has_surface_mesh("second face sheet envelope triangles")
