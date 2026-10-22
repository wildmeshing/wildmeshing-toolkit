"""Surface tracking through the bindings: histories as bytes on the memory route, as files on the
file route, and wm.read_history / wm.write_history between the two.

Also runnable with Python's standard-library unittest runner; NumPy only, synthetic meshes."""

import os
from pathlib import Path
import struct
import tempfile
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


REMESH = {"length_rel": 0.2, "max_iterations": 2, "num_threads": 1}


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
        with self.assertRaisesRegex(RuntimeError, "only info and transfer"):
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


if __name__ == "__main__":
    unittest.main()
