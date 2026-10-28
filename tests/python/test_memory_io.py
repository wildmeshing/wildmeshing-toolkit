"""In-memory bindings; also runnable with Python's standard-library unittest runner."""

import copy
import gc
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np
import wildmeshing as wm


def triangle():
    return {
        "V": np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float64),
        "cells": {"triangle": {"connectivity": np.array([[0, 1, 2]], dtype=np.int64)}},
    }


def tetrahedron():
    return {
        "V": np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64),
        "cells": {"tetrahedron": {"connectivity": np.array([[0, 1, 2, 3]], dtype=np.int64)}},
    }


def surface():
    mesh = tetrahedron()
    mesh["cells"] = {"triangle": {"connectivity": np.array(
        [[1, 2, 3], [0, 3, 2], [0, 1, 3], [0, 2, 1]], dtype=np.int64)}}
    return mesh


def curves():
    return {
        "V": np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=np.float64),
        "cells": {"edge": {"connectivity": np.array(
            [[0, 1], [1, 2], [2, 3], [3, 0]], dtype=np.int64)}},
    }


def tagged(mesh, cell_type):
    mesh["cells"][cell_type]["attributes"] = {
        "tags": {"values": np.ones((1, 1), dtype=np.uint8), "labels": ["tag_0"]}}
    return mesh


class MemoryIO(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="wmtk_python_io_")
        self.previous_cwd = Path.cwd()
        os.chdir(self.directory.name)

    def tearDown(self):
        os.chdir(self.previous_cwd)
        self.directory.cleanup()

    def assert_tree_equal(self, a, b):
        if isinstance(a, np.ndarray):
            self.assertEqual(a.dtype, b.dtype)
            np.testing.assert_array_equal(a, b)
        elif isinstance(a, dict):
            self.assertEqual(a.keys(), b.keys())
            for key in a:
                self.assert_tree_equal(a[key], b[key])
        else:
            self.assertEqual(a, b)

    def run_memory(self, name, options, meshes):
        before, options_before = copy.deepcopy(meshes), copy.deepcopy(options)
        outputs, report = getattr(wm, name)(options, meshes)
        self.assertIn("mesh", outputs)
        self.assertIsInstance(report, dict)
        self.assert_tree_equal(meshes, before)
        self.assertEqual(options, options_before)
        self.assertEqual(list(Path.cwd().iterdir()), [])
        for mesh in outputs.values():
            self.assertEqual(mesh["V"].dtype, np.dtype("float64"))
            self.assertTrue(mesh["V"].flags.owndata)
            for block in mesh["cells"].values():
                self.assertEqual(block["connectivity"].dtype, np.dtype("int64"))
        return outputs, report

    def test_surface_components(self):
        for name in ("qslim", "shortest_edge_collapse", "isotropic_remeshing"):
            with self.subTest(component=name):
                options = dict(input="surface", target_rel=1.0, eps_rel=0.0,
                               num_threads=1, DEBUG_output=True,
                               output="must_not_exist.obj", log_file="must_not_exist.log")
                if name == "isotropic_remeshing":
                    options["max_iterations"] = 0
                self.run_memory(name, options, {"surface": triangle()})

    def test_simwild_tagged(self):
        for cell_type, mesh in (("triangle", triangle()), ("tetrahedron", tetrahedron())):
            with self.subTest(cell_type=cell_type):
                outputs, _ = self.run_memory(
                    "simwild", dict(input="tagged", envelope="boundary", max_iterations=0, num_threads=1,
                                    DEBUG_output=True, write_vtu=True),
                    {"tagged": tagged(mesh, cell_type),
                     "boundary": surface() if cell_type == "tetrahedron" else {
                         "V": mesh["V"].copy(), "cells": {"edge": {"connectivity": np.array(
                             [[0, 1], [1, 2], [2, 0]], dtype=np.int64)}}}})
                field = outputs["mesh"]["cells"][cell_type]["attributes"]["tags"]
                self.assertEqual(field["values"].dtype, np.dtype("uint8"))
                self.assertIn("tag_0", field["labels"])

    def test_triwild(self):
        self.run_memory("triwild", dict(input=["curves"], skip_simplify=True,
                                       max_iterations=0, num_threads=1,
                                       DEBUG_output=True, write_vtu=True),
                        {"curves": curves()})

    def test_tetwild(self):
        self.run_memory("tetwild", dict(input="surface", skip_simplify=True,
                                       max_iterations=0, num_threads=1,
                                       DEBUG_output=True, write_vtu=True),
                        {"surface": surface()})

    def test_offset_and_manifold(self):
        meshes = {"tagged": tagged(tetrahedron(), "tetrahedron")}
        self.run_memory("topological_offset", dict(input="tagged", optimize_offset=False,
                                                  offset_selection="tag_0 & !tag_0",
                                                  check_manifoldness=False, num_threads=1), meshes)
        outputs, _ = self.run_memory("manifold_extraction", dict(input="tagged", tag_selection="tag_0",
                                                               num_threads=1),
                                     {"tagged": tagged(triangle(), "triangle")})
        self.assertIn("surface", outputs)

    def test_prismatic_exact_int64(self):
        mesh = tetrahedron()
        ids = np.array([2**53 + 1, 2**53 + 3, 2**53 + 5, 2**53 + 7], dtype=np.int64)
        corr = np.array([-1, -1, -1, 2**53 + 1], dtype=np.int64)
        mesh["vertex_attributes"] = {
            "labels": np.array([1, 0, 0, 2], dtype=np.int64),
            "vid": ids, "corr_input_vid": corr}
        mesh["cells"]["tetrahedron"]["attributes"] = {
            "tag_0": np.zeros((1, 1), dtype=np.uint8),
            "offset_tag": np.ones((1, 1), dtype=np.int64)}
        outputs, _ = self.run_memory("prismatic_mesh", dict(input="construction", iterations=0),
                                     {"construction": mesh})
        attributes = outputs["mesh"]["vertex_attributes"]
        np.testing.assert_array_equal(attributes["vid"]["values"][:, 0], ids)
        np.testing.assert_array_equal(attributes["corr_input_vid"]["values"][:, 0], corr)

    def test_c1(self):
        mesh = triangle()
        mesh["vertex_attributes"] = {"is_cone": np.ones(3, dtype=np.uint8)}
        mesh["cells"]["triangle"]["attributes"] = {"dofs": np.zeros((1, 36), dtype=np.float64)}
        outputs, _ = self.run_memory("c1_simplification", dict(surface_mesh="surface", uv_mesh="uv"),
                                     {"surface": mesh, "uv": triangle()})
        self.assertIn("uv", outputs)

    def test_pipeline(self):
        meshes, _ = self.run_memory(
            "simwild", dict(input="surface", input_names=["tag_0"], max_iterations=0,
                            skip_simplify=True, num_threads=1, length_rel=0.5, coarsen_pass=False),
            {"surface": surface()})
        meshes, _ = self.run_memory(
            "topological_offset", dict(input="mesh", envelope="envelope", offset_selection="tag_0",
                                       offset_output_tags=["offset"], optimize_offset=False,
                                       save_offset_correspondence=True, check_manifoldness=False,
                                       num_threads=1), meshes)
        self.run_memory("prismatic_mesh", dict(input="mesh", iterations=0), meshes)

    def test_ordered_multi_input(self):
        left, right = surface(), surface()
        right["V"][:, 0] += 3
        outputs, _ = self.run_memory(
            "simwild", dict(input=["right", "left"], input_names=["right_region", "left_region"],
                            max_iterations=0, skip_simplify=True, num_threads=1,
                            length_rel=0.5, coarsen_pass=False),
            {"left": left, "right": right})
        mesh = outputs["mesh"]
        block = mesh["cells"]["tetrahedron"]
        field = block["attributes"]["tags"]
        centers = mesh["V"][block["connectivity"]].mean(axis=1)
        for label, right_side in (("right_region", True), ("left_region", False)):
            selected = field["values"][:, field["labels"].index(label)] != 0
            self.assertTrue(selected.any())
            self.assertTrue(np.all((centers[selected, 0] > 2) == right_side))

    def test_dispatcher_and_unselected_data(self):
        options = dict(application="qslim", input="surface", target_rel=1.0, eps_rel=0.0)
        meshes, report = wm.wildmeshing(options, {"surface": triangle(), "unused": object()})
        self.assertIn("mesh", meshes)
        self.assertIsInstance(report, dict)
        with self.assertRaisesRegex(RuntimeError, "application"):
            wm.wildmeshing({}, {})
        with self.assertRaisesRegex(RuntimeError, "unknown"):
            wm.wildmeshing({"application": "unknown"}, {})
        with self.assertRaisesRegex(RuntimeError, "Unknown input mesh"):
            wm.qslim({"input": "missing"}, {})

    def test_no_filename_fallback(self):
        Path("surface.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        with self.assertRaisesRegex(RuntimeError, "Unknown input mesh"):
            wm.qslim({"input": "surface.obj"}, {})

    def test_noncontiguous_readonly_arrays_and_output_lifetime(self):
        mesh = triangle()
        mesh["V"] = np.asfortranarray(mesh["V"].astype(np.float32))
        mesh["cells"]["triangle"]["connectivity"] = np.array([[2, 1, 0]], dtype=np.int32)[:, ::-1]
        mesh["V"].flags.writeable = False
        mesh["cells"]["triangle"]["connectivity"].flags.writeable = False
        original = mesh["V"].copy()
        outputs, _ = self.run_memory("qslim", dict(input="surface", target_rel=1.0, eps_rel=0.0),
                                     {"surface": mesh})
        positions = outputs["mesh"]["V"]
        del outputs
        gc.collect()
        positions[0, 0] = 42
        np.testing.assert_array_equal(mesh["V"], original)

    def test_rejected_data(self):
        cases = []
        for connectivity in (np.array([[0.1, 1, 2]], dtype=np.float64),
                             np.array([[2**63, 1, 2]], dtype=np.uint64),
                             np.array([[0, 1, 9]], dtype=np.int64),
                             np.array([[0, 1, 1]], dtype=np.int64),
                             np.array([0, 1, 2], dtype=np.int64)):
            mesh = triangle()
            mesh["cells"]["triangle"]["connectivity"] = connectivity
            cases.append(mesh)
        mesh = triangle()
        mesh["vertex_attributes"] = {"extra": np.ones((3, 1), dtype=np.float64)}
        cases.append(mesh)
        mesh = triangle()
        mesh["vertex_attributes"] = {"extra": np.ones((3, 1), dtype=np.float32)}
        cases.append(mesh)
        mesh = triangle()
        mesh["vertices"] = mesh["V"]
        cases.append(mesh)
        mesh = triangle()
        mesh["V"] = mesh["V"][:, :2]
        cases.append(mesh)
        for i, mesh in enumerate(cases):
            with self.subTest(case=i), self.assertRaises((ValueError, TypeError, RuntimeError)):
                wm.qslim({"input": "surface"}, {"surface": mesh})
        with self.assertRaisesRegex(RuntimeError, "corr_input_vid"):
            wm.prismatic_mesh({"input": "construction"}, {"construction": tetrahedron()})
        self.assertEqual(list(Path.cwd().iterdir()), [])

    def test_msh_roundtrip(self):
        mesh = surface()
        mesh["vertex_attributes"] = {
            "source_id": {"values": np.arange(4, dtype=np.int64)[:, None], "labels": []}}
        mesh["cells"]["triangle"]["attributes"] = {
            "tags": {"values": np.array([[1, 1], [1, 0], [0, 1], [0, 0]], dtype=np.uint8),
                     "labels": ["first", "overlap"]},
            "dofs": {"values": np.arange(144, dtype=np.float64).reshape(4, 36), "labels": []}}
        wm.write_mesh(Path("mesh.msh"), mesh)
        loaded = wm.read_mesh(Path("mesh.msh"))
        mesh.setdefault("vertex_attributes", {})
        self.assert_tree_equal(mesh, loaded)

    def test_legacy_file_interfaces(self):
        Path("surface.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        options = dict(input="surface.obj", target_rel=1.0, eps_rel=0.0, num_threads=1)
        wm.wildmeshing(dict(options, application="qslim", output="old.obj", report="old.json"))
        wm.qslim(dict(options, output="direct.obj"))
        outputs, report = wm.qslim(dict(options, input="surface"), {"surface": triangle()})
        self.assertEqual(Path("old.obj").read_bytes(), Path("direct.obj").read_bytes())
        np.testing.assert_array_equal(wm.read_mesh("old.obj")["V"], outputs["mesh"]["V"])
        file_report = json.loads(Path("old.json").read_text())
        file_report.pop("time_sec")
        report.pop("time_sec")
        self.assertEqual(file_report, report)

    def test_legacy_dispatch_errors_and_expressions(self):
        for options in ({}, {"application": "this_application_does_not_exist"}):
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                wm.wildmeshing(options)
        expression = wm.Expression("a & !b", ["a", "b"])
        self.assertTrue(expression.eval({"a"}))
        self.assertFalse(expression.eval({"a", "b"}))


if __name__ == "__main__":
    unittest.main()
