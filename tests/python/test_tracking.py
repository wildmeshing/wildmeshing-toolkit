"""Tracking through the bindings: histories of surfaces and of curves as bytes on the memory
route, as files on the file route, and wm.read_history / wm.write_history between the two; and
the maps they describe, wm.Tracking and wm.CurveTracking, on NumPy arrays of points.

Also runnable with Python's standard-library unittest runner; NumPy only, synthetic meshes."""

import os
from pathlib import Path
import struct
import tempfile
import threading
import unittest

import numpy as np
import wildmeshing as wm

# The component that maps through a history.
TRACKING = "tracking"


def sphere(n_lat=8, n_lon=16):
    """A closed latitude-longitude sphere, outward oriented, with poles."""
    V = [[0.0, 0.0, 1.0]]
    for i in range(1, n_lat):
        theta = np.pi * i / n_lat
        for j in range(n_lon):
            phi = 2 * np.pi * j / n_lon
            V.append([np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi), np.cos(theta)])
    V.append([0.0, 0.0, -1.0])
    ring = lambda i, j: 1 + (i - 1) * n_lon + j % n_lon
    south = len(V) - 1
    F = [[0, ring(1, j), ring(1, j + 1)] for j in range(n_lon)]
    for i in range(1, n_lat - 1):
        for j in range(n_lon):
            a, b, c, d = ring(i, j), ring(i + 1, j), ring(i + 1, j + 1), ring(i, j + 1)
            F += [[a, b, c], [a, c, d]]
    F += [[south, ring(n_lat - 1, j + 1), ring(n_lat - 1, j)] for j in range(n_lon)]
    return {"V": np.array(V, dtype=np.float64),
            "cells": {"triangle": {"connectivity": np.array(F, dtype=np.int64)}}}


def write_obj(path, mesh):
    with open(path, "w") as f:
        for v in mesh["V"]:
            f.write("v {:.17g} {:.17g} {:.17g}\n".format(*v))
        for t in mesh["cells"]["triangle"]["connectivity"]:
            f.write("f {} {} {}\n".format(*(t + 1)))


def curves(n=60, m=30):
    """triwild's curves: a circle and an open wave crossing it twice, as edge meshes."""
    a = 2 * np.pi * np.arange(n) / n
    circle = {"V": np.stack([np.cos(a), np.sin(a), np.zeros(n)], axis=1),
              "cells": {"edge": {"connectivity": np.stack(
                  [np.arange(n), (np.arange(n) + 1) % n], axis=1).astype(np.int64)}}}
    x = -1.6 + 3.2 * np.arange(m + 1) / m
    wave = {"V": np.stack([x, 0.3 * np.sin(6 * x) + 0.1, np.zeros(m + 1)], axis=1),
            "cells": {"edge": {"connectivity": np.stack(
                [np.arange(m), np.arange(1, m + 1)], axis=1).astype(np.int64)}}}
    return {"circle": circle, "wave": wave}


REMESH = {"length_rel": 0.2, "max_iterations": 2, "num_threads": 1}
TRIWILD = {"input": ["circle", "wave"], "preserve_topology": True, "length_rel": 0.05,
           "max_iterations": 3, "num_threads": 1}


class Tracking(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="wmtk_python_tracking_")
        self.previous_cwd = Path.cwd()
        os.chdir(self.directory.name)

    def tearDown(self):
        os.chdir(self.previous_cwd)
        self.directory.cleanup()

    def assert_history(self, history):
        """Bytes of a surface history file: the magic, version 7, three corners a face."""
        self.assertIsInstance(history, bytes)
        self.assertEqual(history[:8], b"WMTKHIST")
        self.assertEqual(history[8:13], b"\x07\x00\x00\x00\x03")

    def test_memory_route_returns_history_bytes(self):
        # Every producer: the history in the report, and no file anywhere.
        runs = {
            "isotropic_remeshing": dict(REMESH),
            "qslim": {"target_rel": 0.5, "num_threads": 1},
            "shortest_edge_collapse": {"target_rel": 0.5, "num_threads": 1},
            "tetwild": {"skip_simplify": True, "max_iterations": 1, "num_threads": 1},
        }
        for name, options in runs.items():
            with self.subTest(component=name):
                outputs, report = getattr(wm, name)(
                    {**options, "input": "in", "history_output": "recorded.wmtkhist"},
                    {"in": sphere()})
                self.assertIn("mesh", outputs)
                self.assert_history(report["history"])
                self.assertEqual(list(Path.cwd().iterdir()), [])
        # Without history_output nothing is recorded.
        _, report = wm.qslim({"input": "in", "target_rel": 0.5}, {"in": sphere()})
        self.assertNotIn("history", report)

    def test_triwild_returns_a_curve_history(self):
        # A curve history: version 7, two corners a face, in the report and in no file.
        outputs, report = wm.triwild({**TRIWILD, "history_output": "x"}, curves())
        self.assertIn("mesh", outputs)
        history = report["history"]
        self.assertEqual(history[:13], b"WMTKHIST\x07\x00\x00\x00\x02")
        self.assertEqual(list(Path.cwd().iterdir()), [])
        wm.write_history("curves.wmtkhist", history)
        self.assertEqual(wm.read_history("curves.wmtkhist"), history)
        # tracking reports on it in memory: no warning under preserve_topology, the wave's two
        # ends at output vertices, the round trip exact.
        _, info = getattr(wm, TRACKING)({"history": history, "operation": "info"}, {})
        self.assertEqual(info["build_warnings"]["total"], 0)
        self.assertEqual(info["feature_points"], 2)
        self.assertEqual(info["feature_points_at_output_vertex"], 2)
        self.assertEqual(info["round_trip_lost"], 0)
        self.assertLess(info["round_trip_max_displacement"], 1e-8)
        # features: the feature points in the report, an input and an output vertex each, and
        # no file; map is refused.
        _, features = getattr(wm, TRACKING)({"history": history, "operation": "features"}, {})
        pairs = np.array(features["feature_points"], dtype=np.int64)
        self.assertEqual(pairs.shape, (2, 2))
        self.assertTrue((pairs[:, 1] >= 0).all())
        self.assertTrue((pairs[:, 1] < len(outputs["mesh"]["V"])).all())
        self.assertEqual([p.name for p in Path.cwd().iterdir()], ["curves.wmtkhist"])
        with self.assertRaisesRegex(RuntimeError, "only info, transfer, features and volume"):
            getattr(wm, TRACKING)({"history": history, "operation": "map"}, {})

    def test_write_and_read_history(self):
        _, report = wm.isotropic_remeshing(
            {**REMESH, "input": "in", "history_output": "x"}, {"in": sphere()})
        history = report["history"]
        wm.write_history("h.wmtkhist", history)
        self.assertEqual(Path("h.wmtkhist").read_bytes(), history)
        self.assertEqual(wm.read_history("h.wmtkhist"), history)
        # Bytes that are no history are refused, and nothing is written.
        with self.assertRaises(RuntimeError):
            wm.write_history("bad.wmtkhist", b"not a history at all")
        self.assertFalse(Path("bad.wmtkhist").exists())
        write_obj("mesh.obj", sphere())
        with self.assertRaises(RuntimeError):
            wm.read_history("mesh.obj")
        with self.assertRaises(RuntimeError):
            wm.read_history("missing.wmtkhist")

    def test_file_route_writes_the_history_file(self):
        write_obj("in.obj", sphere())
        wm.isotropic_remeshing({**REMESH, "application": "isotropic_remeshing",
                                "input": str(Path("in.obj").resolve()),
                                "output": str(Path("out.obj").resolve()),
                                "history_output": str(Path("out.wmtkhist").resolve())})
        history = wm.read_history("out.wmtkhist")
        self.assert_history(history)
        self.assertEqual(history, Path("out.wmtkhist").read_bytes())

    def labelled(self):
        """The sphere with a label per face (Int64) and texture coordinates per corner."""
        mesh = sphere()
        F = mesh["cells"]["triangle"]["connectivity"]
        uv = mesh["V"][F][:, :, :2].reshape(len(F), 6)
        mesh["cells"]["triangle"]["attributes"] = {
            "label": {"values": (np.arange(len(F)) % 5).astype(np.int64).reshape(-1, 1)},
            "uv": {"values": uv}}
        return mesh

    def assert_mesh_equal(self, a, b):
        np.testing.assert_array_equal(a["V"], b["V"])
        self.assertEqual(a["cells"].keys(), b["cells"].keys())
        for cell, block in a["cells"].items():
            np.testing.assert_array_equal(block["connectivity"], b["cells"][cell]["connectivity"])
            for name, field in block.get("attributes", {}).items():
                other = b["cells"][cell]["attributes"][name]
                self.assertEqual(field["values"].dtype, other["values"].dtype)
                np.testing.assert_array_equal(field["values"], other["values"])

    def test_tracking_on_the_memory_route(self):
        tracking = getattr(wm, TRACKING)
        remeshed, report = wm.isotropic_remeshing(
            {**REMESH, "input": "in", "history_output": "x"}, {"in": sphere()})
        history = report["history"]

        # info: the report of the map the bytes describe.
        outputs, info = tracking({"history": history, "operation": "info"}, {})
        self.assertEqual(outputs, {})
        self.assertGreater(info["edits"], 0)
        self.assertEqual(info["build_warnings"]["total"], 0)
        self.assertLess(info["round_trip_max_displacement"], 1e-8)

        # transfer: the fields of the mesh given onto the output, returned as a mesh in the
        # conventions of the mesh files -- a label per face, uv six columns -- that write_mesh
        # and read_mesh keep.
        outputs, transferred = tracking(
            {"history": history, "operation": "transfer", "input": "in"}, {"in": self.labelled()})
        out = outputs["mesh"]
        triangles = out["cells"]["triangle"]
        np.testing.assert_array_equal(
            triangles["connectivity"], remeshed["mesh"]["cells"]["triangle"]["connectivity"])
        fields = triangles["attributes"]
        self.assertEqual(fields["uv"]["values"].shape, (len(triangles["connectivity"]), 6))
        self.assertEqual(fields["label"]["values"].dtype, np.dtype("int64"))
        source = fields["input_face"]["values"][:, 0]
        self.assertTrue((source >= 0).all())
        np.testing.assert_array_equal(fields["label"]["values"][:, 0], source % 5)
        self.assertEqual(transferred["to_input_warnings"]["total"], 0)
        wm.write_mesh("transferred.msh", out)
        self.assert_mesh_equal(wm.read_mesh("transferred.msh"), out)
        os.remove("transferred.msh")

        # What the memory route refuses: a file name for the history -- even of a file there --
        # a transfer without its input, map.
        wm.write_history("h.wmtkhist", history)
        with self.assertRaisesRegex(RuntimeError, "not a file name"):
            tracking({"history": "h.wmtkhist", "operation": "info"}, {})
        with self.assertRaisesRegex(RuntimeError, "needs input"):
            tracking({"history": history, "operation": "transfer"}, {"in": self.labelled()})
        with self.assertRaisesRegex(RuntimeError, "only info, transfer, features and volume"):
            tracking({"history": history, "operation": "map"}, {})
        self.assertEqual([p.name for p in Path.cwd().iterdir()], ["h.wmtkhist"])

    def test_damaged_history_raises(self):
        # Damaged bytes raise a Python exception: cut short anywhere, or a byte changed.
        tracking = getattr(wm, TRACKING)
        _, report = wm.qslim({"input": "in", "target_rel": 0.5, "history_output": "x"},
                             {"in": sphere()})
        history = report["history"]
        tracking({"history": history, "operation": "info", "scaffold": "ring",
                  "smoothing": "barycentric"}, {})
        rng = np.random.default_rng(1)
        raised = 0
        for n in sorted(set(rng.integers(0, len(history), 40).tolist())):
            with self.assertRaises(RuntimeError):
                tracking({"history": history[:n], "operation": "info"}, {})
        for _ in range(60):
            damaged = bytearray(history)
            damaged[int(rng.integers(0, len(damaged)))] ^= int(rng.integers(1, 256))
            try:
                tracking({"history": bytes(damaged), "operation": "info", "scaffold": "ring",
                          "smoothing": "barycentric"}, {})
            except RuntimeError:
                raised += 1
        self.assertGreater(raised, 0)
        # One that passed every bound and crashed the map: an embed without the positions of
        # the vertices it brings.
        varint = lambda n: bytes([n])  # below 128, one byte
        point = lambda *p: struct.pack("<3d", *p)
        crafted = (b"WMTKHIST" + struct.pack("<IB", 7, 3)
                   + varint(3) + b"\x01" + point(0, 0, 0) + b"\x01" + point(1, 0, 0)
                   + b"\x01" + point(0, 1, 0)
                   + varint(1) + b"\x01" + bytes([0, 1, 2]) + varint(0)
                   + varint(1) + bytes([9, 1, 5, 6, 7, 1, 0, 0]) + varint(0)
                   + bytes([0, 0, 0]))
        with self.assertRaisesRegex(RuntimeError, "is corrupt"):
            tracking({"history": crafted, "operation": "info"}, {})

    def test_transfer_onto_a_tet_mesh_in_memory(self):
        tracking = getattr(wm, TRACKING)
        meshed, report = wm.tetwild(
            {"input": "in", "skip_simplify": True, "max_iterations": 1, "num_threads": 1,
             "history_output": "x"}, {"in": sphere()})
        outputs, _ = tracking(
            {"history": report["history"], "operation": "transfer", "input": "in",
             "tet_mesh": "tets"},
            {"in": self.labelled(), "tets": meshed["mesh"]})
        tet = outputs["tet_mesh"]
        np.testing.assert_array_equal(tet["V"], meshed["mesh"]["V"])
        np.testing.assert_array_equal(
            tet["cells"]["tetrahedron"]["connectivity"],
            meshed["mesh"]["cells"]["tetrahedron"]["connectivity"])
        # The tracked surface on the tet mesh's own vertices, every face where it came from.
        surface = outputs["mesh"]["cells"]["triangle"]
        triangles = tet["cells"]["triangle"]
        # tetwild writes no triangles of its own: each tracked face is a row, in the surface's
        # order, its corners at the surface's corners.
        np.testing.assert_array_equal(
            tet["V"][triangles["connectivity"]],
            outputs["mesh"]["V"][surface["connectivity"]])
        self.assertTrue((triangles["attributes"]["input_face"]["values"] >= 0).all())
        np.testing.assert_array_equal(
            triangles["attributes"]["label"]["values"][:, 0],
            triangles["attributes"]["input_face"]["values"][:, 0] % 5)
        wm.write_mesh("tet.msh", tet)
        self.assert_mesh_equal(wm.read_mesh("tet.msh"), tet)

    def test_tracking_on_the_file_route(self):
        # The same through files: the history the producer wrote, the input written by
        # write_mesh with its fields, the output read back by read_mesh.
        tracking = getattr(wm, TRACKING)
        write_obj("in.obj", sphere())
        wm.isotropic_remeshing({**REMESH, "application": "isotropic_remeshing",
                                "input": str(Path("in.obj").resolve()),
                                "output": str(Path("out.obj").resolve()),
                                "history_output": str(Path("out.wmtkhist").resolve())})
        wm.write_mesh("labelled.msh", self.labelled())
        tracking({"application": TRACKING, "history": str(Path("out.wmtkhist").resolve()),
                  "operation": "transfer", "input": str(Path("labelled.msh").resolve()),
                  "output": str(Path("transferred.msh").resolve())})
        out = wm.read_mesh("transferred.msh")
        fields = out["cells"]["triangle"]["attributes"]
        self.assertEqual(fields["uv"]["values"].shape[1], 6)
        np.testing.assert_array_equal(
            fields["label"]["values"][:, 0], fields["input_face"]["values"][:, 0] % 5)
        # Bytes for the history on the file route are refused: there it names a file.
        with self.assertRaisesRegex(RuntimeError, "history is a file on the file route"):
            tracking({"application": TRACKING, "history": wm.read_history("out.wmtkhist")})

    def test_bytes_in_options_stay_bytes(self):
        # pybind11_json would turn bytes into base64 text; the bindings pass them as binary,
        # which a string option refuses instead of taking as a file name.
        with self.assertRaises(RuntimeError):
            wm.qslim({"input": "in", "target_rel": 0.5, "history_output": b"recorded"},
                     {"in": sphere()})
        self.assertEqual(list(Path.cwd().iterdir()), [])


def random_bary(rng, n, k):
    """n points' barycentric coordinates in a k-corner face, inside it."""
    b = rng.random((n, k)) + 0.05
    return b / b.sum(axis=1, keepdims=True)


class Classes(unittest.TestCase):
    """wm.Tracking and wm.CurveTracking on histories the memory route returns."""

    @classmethod
    def setUpClass(cls):
        cls.surface_out, report = wm.isotropic_remeshing(
            {**REMESH, "input": "in", "history_output": "x"}, {"in": sphere()})
        cls.surface = report["history"]
        cls.tets, report = wm.tetwild(
            {"input": "in", "skip_simplify": True, "max_iterations": 1, "num_threads": 1,
             "history_output": "x"}, {"in": sphere()})
        cls.tet = report["history"]
        cls.curves_out, report = wm.triwild({**TRIWILD, "history_output": "x"}, curves())
        cls.curves = report["history"]

    def setUp(self):
        self.rng = np.random.default_rng(3)

    def test_surface_round_trip_and_info(self):
        t = wm.Tracking(self.surface)
        self.assertFalse(t.output_by_vertices)
        self.assertFalse(t.input_by_vertices)
        np.testing.assert_array_equal(
            t.input_faces, sphere()["cells"]["triangle"]["connectivity"])
        out_F = self.surface_out["mesh"]["cells"]["triangle"]["connectivity"]
        np.testing.assert_array_equal(t.output_triangles, out_F)
        self.assertEqual(t.input_triangles.shape, t.input_faces.shape)
        # Input points to the output, faces of the output mesh, and back where they were.
        n = 500
        ids = self.rng.integers(0, len(t.input_faces), n)
        bary = random_bary(self.rng, n, 3)
        out_ids, out_bary = t.to_output(ids, bary)
        self.assertEqual(out_ids.shape, (n,))
        self.assertEqual(out_ids.dtype, np.int64)
        self.assertEqual(out_bary.shape, (n, 3))
        self.assertTrue(((out_ids >= 0) & (out_ids < len(out_F))).all())
        back_ids, back_bary = t.to_input(out_ids, out_bary)
        np.testing.assert_array_equal(back_ids, ids)
        np.testing.assert_allclose(back_bary, bary, atol=1e-8)
        # What info says of the same map: its warnings, and the output centroids' round trip
        # and drift, here computed through the class.
        _, info = getattr(wm, TRACKING)({"history": self.surface, "operation": "info"}, {})
        self.assertEqual(t.build_warnings, info["build_warnings"])
        k = len(out_F)
        self.assertEqual(info["centroids"], k)
        centroids = np.full((k, 3), 1 / 3)
        in_ids, in_bary = t.to_input(np.arange(k), centroids)
        self.assertEqual(int((in_ids < 0).sum()), info["round_trip_lost"])
        V_in, F_in = sphere()["V"], t.input_faces
        V_out = self.surface_out["mesh"]["V"]
        at_out = V_out[out_F].mean(axis=1)
        at_in = np.einsum("nk,nkd->nd", in_bary, V_in[F_in[in_ids]])
        edges = np.linalg.norm(V_out[out_F] - V_out[np.roll(out_F, -1, axis=1)], axis=2).mean()
        drift = np.linalg.norm(at_out - at_in, axis=1)
        self.assertAlmostEqual(info["drift"]["mean_length"], edges, places=12)
        self.assertAlmostEqual(info["drift"]["lengths"]["mean"], drift.mean() / edges, places=9)
        self.assertAlmostEqual(info["drift"]["lengths"]["max"], drift.max() / edges, places=9)

    def test_tet_output_by_vertices(self):
        # tetwild's output names a point on its tracked surface by three of its vertices.
        t = wm.Tracking(self.tet)
        self.assertTrue(t.output_by_vertices)
        n = 300
        ids = self.rng.integers(0, len(t.input_faces), n)
        bary = random_bary(self.rng, n, 3)
        out_ids, out_bary = t.to_output(ids, bary)
        self.assertEqual(out_ids.shape, (n, 3))
        self.assertTrue(((out_ids >= 0) & (out_ids < len(self.tets["mesh"]["V"]))).all())
        tracked = {tuple(sorted(f)) for f in t.output_triangles.tolist()}
        self.assertTrue(all(tuple(sorted(f)) in tracked for f in out_ids.tolist()))
        # Back, the vertices in another order, the coordinates following them.
        order = [2, 0, 1]
        back_ids, back_bary = t.to_input(out_ids[:, order], out_bary[:, order])
        np.testing.assert_array_equal(back_ids, ids)
        np.testing.assert_allclose(back_bary, bary, atol=1e-8)
        # Faces, not triplets, are refused for this output.
        with self.assertRaisesRegex(ValueError, r"\(n, 3\)"):
            t.to_input(out_ids[:, 0], out_bary)

    def test_curve_round_trip_and_info(self):
        t = wm.CurveTracking(self.curves)
        self.assertTrue(t.output_by_vertices)
        self.assertEqual(t.input_segments.shape, (60 + 30, 2))
        n = 400
        ids = self.rng.integers(0, len(t.input_segments), n)
        bary = random_bary(self.rng, n, 2)
        out_ids, out_bary = t.to_output(ids, bary)
        self.assertEqual(out_ids.shape, (n, 2))
        edges = {tuple(sorted(e)) for e in t.output_segments.tolist()}
        self.assertTrue(all(tuple(sorted(e)) in edges for e in out_ids.tolist()))
        back_ids, back_bary = t.to_input(out_ids[:, ::-1], out_bary[:, ::-1])
        np.testing.assert_array_equal(back_ids, ids)
        np.testing.assert_allclose(back_bary, bary, atol=1e-8)
        # The feature points are those features reports, and info agrees on the map.
        tracking = getattr(wm, TRACKING)
        _, features = tracking({"history": self.curves, "operation": "features"}, {})
        np.testing.assert_array_equal(t.feature_points(), np.array(features["feature_points"]))
        self.assertEqual(t.feature_points().shape, (2, 2))
        _, info = tracking({"history": self.curves, "operation": "info"}, {})
        self.assertEqual(t.build_warnings, info["build_warnings"])
        self.assertEqual(info["midpoints"], len(t.output_segments))

    def test_unmappable_points(self):
        # -1 for no face, either way; an id beyond the input is no face either, and the
        # warnings of the call count both.
        t = wm.Tracking(self.surface)
        ids = np.array([0, -1, len(t.input_faces) + 5, 1])
        bary = np.full((4, 3), 1 / 3)
        out_ids, out_bary, warnings = t.to_output(ids, bary, warnings=True)
        np.testing.assert_array_equal(out_ids < 0, [False, True, True, False])
        self.assertTrue(np.isnan(out_bary[1]).all() and np.isnan(out_bary[2]).all())
        self.assertEqual(warnings["unlocated"], 2)
        self.assertEqual(warnings["total"], 2)
        self.assertEqual(len(t.to_output(ids, bary)), 2)
        back_ids, _ = t.to_input(out_ids, np.nan_to_num(out_bary))
        np.testing.assert_array_equal(back_ids, [0, -1, -1, 1])
        c = wm.CurveTracking(self.curves)
        out_ids, out_bary, warnings = c.to_output(np.array([-1, 0]), np.full((2, 2), 0.5),
                                                  warnings=True)
        np.testing.assert_array_equal(out_ids[0], [-1, -1])
        self.assertTrue((out_ids[1] >= 0).all())
        self.assertEqual(warnings["unlocated"], 1)
        back_ids, _ = c.to_input(np.array([[-1, -1], [0, 10 ** 6]]), np.full((2, 2), 0.5))
        np.testing.assert_array_equal(back_ids, [-1, -1])

    def test_wrong_shapes_and_dtypes_raise(self):
        t = wm.Tracking(self.surface)
        c = wm.CurveTracking(self.curves)
        ids, bary = np.array([0, 1]), np.full((2, 3), 1 / 3)
        with self.assertRaises(TypeError):
            t.to_output(ids.astype(np.float64), bary)
        with self.assertRaises(TypeError):
            t.to_output([0, 1], bary)
        with self.assertRaises(TypeError):
            t.to_output(ids, bary.astype(np.int64))
        with self.assertRaisesRegex(ValueError, r"\(n,\)"):
            t.to_output(np.zeros((2, 3), dtype=np.int64), bary)
        with self.assertRaisesRegex(ValueError, r"\(n, 3\)"):
            t.to_output(ids, np.full((2, 2), 0.5))
        with self.assertRaisesRegex(ValueError, r"\(n, 3\)"):
            t.to_output(ids, np.full((3, 3), 1 / 3))
        with self.assertRaisesRegex(ValueError, "-1 for none"):
            t.to_output(np.array([0, -2]), bary)
        with self.assertRaisesRegex(ValueError, r"\(n, 2\)"):
            c.to_input(np.array([0, 1]), np.full((2, 2), 0.5))
        with self.assertRaisesRegex(ValueError, r"\(n, 2\)"):
            c.to_output(np.array([0, 1]), bary)
        # Histories: not bytes, the other kind, damaged, or options that do not exist.
        with self.assertRaises(TypeError):
            wm.Tracking("history.wmtkhist")
        with self.assertRaisesRegex(ValueError, "CurveTracking"):
            wm.Tracking(self.curves)
        with self.assertRaisesRegex(ValueError, "with Tracking"):
            wm.CurveTracking(self.surface)
        with self.assertRaisesRegex(RuntimeError, "is corrupt|truncated"):
            wm.Tracking(self.surface[: len(self.surface) // 2])
        with self.assertRaisesRegex(RuntimeError, "is not a history"):
            wm.CurveTracking(b"not a history at all")
        with self.assertRaisesRegex(ValueError, "smoothing"):
            wm.Tracking(self.surface, smoothing="sharpen")
        with self.assertRaisesRegex(ValueError, "scaffold"):
            wm.Tracking(self.surface, scaffold="sphere")

    def test_load_and_bytes_interoperate(self):
        # A history file, written by write_history or by the file route, loads as its bytes do,
        # and read_history gives bytes the constructors take.
        with tempfile.TemporaryDirectory(prefix="wmtk_python_tracking_classes_") as d:
            surface, curve = Path(d) / "surface.wmtkhist", Path(d) / "curves.wmtkhist"
            wm.write_history(surface, self.surface)
            wm.write_history(curve, self.curves)
            ids = self.rng.integers(0, 50, 100)
            bary = random_bary(self.rng, 100, 3)
            for t in [wm.Tracking.load(surface), wm.Tracking(wm.read_history(surface)),
                      wm.Tracking.load(str(surface), smoothing="flatten", scaffold="box")]:
                for a, b in zip(t.to_output(ids, bary),
                                wm.Tracking(self.surface).to_output(ids, bary)):
                    np.testing.assert_array_equal(a, b)
            bary2 = random_bary(self.rng, 100, 2)
            for a, b in zip(wm.CurveTracking.load(curve).to_output(ids, bary2),
                            wm.CurveTracking(wm.read_history(curve)).to_output(ids, bary2)):
                np.testing.assert_array_equal(a, b)
            with self.assertRaisesRegex(ValueError, "CurveTracking"):
                wm.Tracking.load(curve)
            with self.assertRaises(RuntimeError):
                wm.Tracking.load(Path(d) / "missing.wmtkhist")

    def test_threads_map_through_one_object(self):
        # Mapping releases the GIL and never writes to the map: two threads mapping through one
        # object get what one call alone gets.
        surface, curves = wm.Tracking(self.surface), wm.CurveTracking(self.curves)
        for t, k, n_ids in [(surface, 3, len(surface.input_faces)),
                            (curves, 2, len(curves.input_segments))]:
            batches = [(self.rng.integers(0, n_ids, 3000), random_bary(self.rng, 3000, k))
                       for _ in range(2)]
            serial = [t.to_output(ids, bary) for ids, bary in batches]
            results = [None, None]

            def work(i):
                for _ in range(5):
                    results[i] = t.to_output(*batches[i])

            threads = [threading.Thread(target=work, args=(i,)) for i in range(2)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            for got, expected in zip(results, serial):
                for a, b in zip(got, expected):
                    np.testing.assert_array_equal(a, b)


if __name__ == "__main__":
    unittest.main()
