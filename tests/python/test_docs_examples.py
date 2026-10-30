"""Run the documentation's Python and JSON examples (docs/examples), as a reader would.

Each Python example runs as a script, in a directory of its own, with the example meshes in
$WMTK_DATA. Each JSON example is copied into a directory of its own with the example meshes it
names, and run with $WMTK_APP -j. A directory of JSON files is a chain: its files (with any other
file there, say a points file) are copied into one directory and run in name order, each reading
what the ones before it wrote. All must finish without an error. An example that a test of its own
runs (a first line `# docs-example: tested by <test>`) is skipped here.

Registered in ctest as doc_examples_python_json (docs/examples/CMakeLists.txt). By hand, from the
repository root:

    WMTK_APP=build/app/wmtk_app PYTHONPATH=build/bin \\
        python -m unittest discover -s tests/python -p test_docs_examples.py
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXAMPLES = ROOT / "docs" / "examples"
DATA = Path(os.environ.get("WMTK_DATA", ROOT / "data" / "models"))
APP = os.environ.get("WMTK_APP", "")
RUN_DIR = os.environ.get("WMTK_DOC_EXAMPLES_RUN_DIR")


def tested_elsewhere(script):
    with open(script) as f:
        return f.readline().startswith("# docs-example: tested by")


def strings(value):
    """Every string in a JSON value."""
    if isinstance(value, str):
        yield value
    elif isinstance(value, list):
        for v in value:
            yield from strings(v)
    elif isinstance(value, dict):
        for v in value.values():
            yield from strings(v)


class DocsExamples(unittest.TestCase):
    def run_dir(self, name):
        base = Path(RUN_DIR) if RUN_DIR else Path(tempfile.mkdtemp())
        d = base / name
        shutil.rmtree(d, ignore_errors=True)
        d.mkdir(parents=True)
        return d

    def test_python(self):
        scripts = sorted(EXAMPLES.glob("python/*.py"))
        self.assertTrue(scripts)
        env = dict(os.environ, WMTK_DATA=str(DATA))
        for script in scripts:
            if tested_elsewhere(script):
                continue
            with self.subTest(script=script.name):
                cwd = self.run_dir("python_" + script.stem)
                r = subprocess.run([sys.executable, str(script)], cwd=cwd, env=env,
                                   capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stdout[-4000:] + r.stderr[-4000:])

    def run_configs(self, configs, files, cwd):
        for f in files:
            shutil.copy(f, cwd)
        for config in configs:
            for s in strings(json.loads(config.read_text())):
                if (DATA / s).is_file() and not (cwd / s).exists():
                    shutil.copy(DATA / s, cwd / s)
        for config in configs:
            r = subprocess.run([APP, "-j", config.name], cwd=cwd, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0,
                             config.name + "\n" + r.stdout[-4000:] + r.stderr[-4000:])
            self.assertNotIn("has no option", r.stdout + r.stderr)

    @unittest.skipUnless(APP, "WMTK_APP is not set")
    def test_json(self):
        configs = sorted(EXAMPLES.glob("json/*.json"))
        self.assertTrue(configs)
        for config in configs:
            with self.subTest(config=config.name):
                self.run_configs([config], [config], self.run_dir("json_" + config.stem))

    @unittest.skipUnless(APP, "WMTK_APP is not set")
    def test_json_chains(self):
        for chain in sorted(d for d in EXAMPLES.glob("json/*") if d.is_dir()):
            with self.subTest(chain=chain.name):
                files = sorted(f for f in chain.iterdir() if f.is_file())
                configs = [f for f in files if f.suffix == ".json"]
                self.run_configs(configs, files, self.run_dir("json_" + chain.name))


if __name__ == "__main__":
    unittest.main()
