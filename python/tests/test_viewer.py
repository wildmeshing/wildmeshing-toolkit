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


def test_attributes_are_listed_once_per_column_with_their_range():
    mesh = stage()["mesh"]
    mesh["vertex_attributes"] = {"distance": {"values": np.r_[np.nan, np.arange(13.0)]},
                                 "unset": {"values": np.full((14, 1), np.nan)}}
    mesh["cells"]["triangle"]["attributes"]["uv"] = {"values": np.arange(4.0).reshape(2, 2)}
    found = {a["label"]: a for a in viewer._attributes(mesh)}
    assert set(found) == {"vertex distance", "vertex unset", "tetrahedron quality",
                          "triangle uv[0]", "triangle uv[1]"}  # the tags are not attributes
    assert found["vertex distance"]["range"] == (0.0, 12.0)
    assert "1 of 14 not finite" in found["vertex distance"]["text"]
    assert found["vertex unset"]["range"] is None  # nothing to colour by
    assert list(found["triangle uv[1]"]["values"]) == [1.0, 3.0]


def test_colour_by_puts_one_quantity_on_the_layers_it_lives_on():
    polyscope.init("openGL_mock")
    mesh = stage()["mesh"]
    mesh["vertex_attributes"] = {"distance": {"values": np.arange(14.0)}}
    targets = []
    viewer._stage_items(polyscope, "colour", {"mesh": mesh}, (np.zeros(3), 1.0), targets)
    found = {a["label"]: a for a in viewer._attributes(mesh)}
    viewer._colour_by(targets, found["vertex distance"])
    assert all(t["quantity"] == "vertex distance" for t in targets)  # tets, prism, sheet, curve
    viewer._colour_by(targets, found["tetrahedron quality"])
    assert {t["kind"] for t in targets if t.get("quantity")} == {"tetrahedron"}
    for t in targets:  # the earlier quantity is gone from polyscope too
        with pytest.raises(Exception):
            t["structure"].remove_quantity("vertex distance", error_if_absent=True)
    viewer._colour_by(targets, None)
    assert not any(t.get("quantity") for t in targets)


def test_offset_correspondence_joins_each_front_vertex_to_its_input_vertex():
    mesh = {"V": V[:4], "cells": {}, "vertex_attributes": {
        "vid": {"values": np.array([[10], [11], [12], [13]])},
        "corr_input_vid": {"values": np.array([[-1], [10], [-1], [12]])}}}
    (d,) = viewer._offset_correspondence(mesh)["directions"]
    assert np.array_equal(d["A"], V[[1, 3]]) and np.array_equal(d["B"], V[[0, 2]])


def test_history_correspondence_maps_both_ways():
    wm = pytest.importorskip("wildmeshing")
    F = np.array([(1, 4, 3), (1, 3, 2), (5, 6, 7), (5, 7, 8), (1, 2, 6), (1, 6, 5), (4, 8, 7),
                  (4, 7, 3), (1, 5, 8), (1, 8, 4), (2, 3, 7), (2, 7, 6)], np.int64) - 1
    box = {"V": V[:8], "cells": {"triangle": {"connectivity": F}}}
    outputs, report = wm.isotropic_remeshing({"input": "box", "eps_rel": 1e-2, "length_rel": 0.2,
                                              "max_iterations": 2, "history_output": "history"},
                                             {"box": box})
    built = viewer._history_correspondence(box, outputs["mesh"], report["history"])
    to_out, to_in = built["directions"]
    assert len(to_out["A"]) == 8 and len(to_in["A"]) == len(outputs["mesh"]["V"])
    assert not np.isnan(to_out["B"]).any() and not np.isnan(to_in["B"]).any()
    assert built["warnings"] == "map warnings: none"
    polyscope.init("openGL_mock")
    viewer._correspondence_layers(polyscope, "box", built, (np.zeros(3), 1.0))
    assert polyscope.has_curve_network("box input -> output")
    assert polyscope.has_point_cloud("box output -> input images")
    with pytest.raises(ValueError):  # not the surface the history starts from
        viewer._history_correspondence(outputs["mesh"], outputs["mesh"], report["history"])
