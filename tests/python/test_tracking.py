"""Surface tracking through the bindings: histories as bytes on the memory route, as files on the
file route, and wm.read_history / wm.write_history between the two.

Also runnable with Python's standard-library unittest runner; NumPy only, synthetic meshes."""

import os
from pathlib import Path
import tempfile
import unittest

import numpy as np
import wildmeshing as wm

# The component that maps through a history.
TRACKING = "surface_map"


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

    def test_bytes_in_options_stay_bytes(self):
        # pybind11_json would turn bytes into base64 text; the bindings pass them as binary,
        # which a string option refuses instead of taking as a file name.
        with self.assertRaises(RuntimeError):
            wm.qslim({"input": "in", "target_rel": 0.5, "history_output": b"recorded"},
                     {"in": sphere()})
        self.assertEqual(list(Path.cwd().iterdir()), [])


if __name__ == "__main__":
    unittest.main()
