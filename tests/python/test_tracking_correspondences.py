"""The tracking tutorial's pipeline (docs/examples/python/tracking_correspondences.py) end to end:
the dense bunny coarsened by isotropic_remeshing, tetrahedralized by tetwild, both recording
their history, and every vertex mapped through the two composed maps and back.

What it checks, with margins: with more than one thread the result changes slightly from run to
run (the schedulers' order is not deterministic), so the counts are ranges, not values. Measured
on the defaults (macOS arm64, 8 threads): 1,041 faces after remeshing; 5,897 tets with 1,070
tracked triangles; one map warning ("replaced": a tracked triangle the filter removed); every
dense vertex mapped onto the coarse surface and all but two onto the tet surface; mean distance
to the image 6e-3 of the diagonal, max 5.4e-2.

Registered in ctest as python_tracking_correspondences, in Release builds only. By hand, from
the repository root:

    PYTHONPATH=build/bin python -m unittest discover -s tests/python \\
        -p test_tracking_correspondences.py
"""

import importlib.util
import os
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "docs" / "examples" / "python" / "tracking_correspondences.py"


def load_example():
    spec = importlib.util.spec_from_file_location("tracking_correspondences", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TrackingCorrespondences(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        example = load_example()
        if not example.DEFAULT_INPUT.exists():
            raise unittest.SkipTest(f"{example.DEFAULT_INPUT} not found (set WMTK_DATA)")
        cls.r = example.run(example.parse_args(["--no-gui"]))

    def test_meshes(self):
        r = self.r
        self.assertEqual(len(r.F_in), 69451)
        self.assertTrue(800 <= len(r.F_s) <= 1400, len(r.F_s))
        self.assertTrue(800 <= len(r.S_t) <= 1500, len(r.S_t))
        self.assertTrue(3000 <= len(r.T_t) <= 10000, len(r.T_t))

    def test_maps_are_bijections(self):
        # The remeshing's map has no warning; tetwild's at most the few tracked triangles the
        # input filter removes.
        self.assertEqual(self.r.remesh_map.build_warnings["total"], 0,
                         self.r.remesh_map.build_warnings)
        w = self.r.tet_map.build_warnings
        self.assertLessEqual(w["total"], 5, w)
        self.assertEqual(w["total"], w["replaced"], w)

    def test_every_point_maps(self):
        s = self.r.stats
        self.assertEqual(s["dense -> simplified"].unmapped, 0)
        self.assertEqual(s["simplified -> dense"].unmapped, 0)
        self.assertEqual(s["tet surface -> dense"].unmapped, 0)
        # The dense vertices on a triangle the filter removed have no image on the tet surface.
        self.assertLessEqual(s["dense -> tet surface"].unmapped, 50)

    def test_images_are_close(self):
        # Within the envelopes on average (eps_rel 2e-2 for the remeshing, 1e-3 for tetwild),
        # with a margin; a point can also slide along the surface, so the max is looser.
        for name, s in self.r.stats.items():
            with self.subTest(direction=name):
                self.assertLess(s.mean, 1.5e-2, name)
                self.assertLess(s.max, 1.5e-1, name)


if __name__ == "__main__":
    unittest.main()
