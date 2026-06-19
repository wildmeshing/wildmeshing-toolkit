"""Subprocess helper: run a single integration-test JSON through the pywmtk binding.

This mirrors what the C++ side does in both ``tests/integration_tests.cpp``
(``load_json`` + ``wmtk_wrapper``) and ``app/main.cpp``:

    1. parse the JSON input file,
    2. inject ``json_input_file`` so components can resolve relative paths,
    3. dispatch on the ``application`` key (done inside ``pywmtk.wmtk``).

It is executed as its own process so that each test gets an isolated working
directory and global state, and so a crash in the native code fails only the
one parametrized case instead of the whole pytest session.

Usage:
    python _run_binding.py <json_input_file> [output_override]
"""

import json
import sys

import pywmtk


def main() -> int:
    json_input_file = sys.argv[1]
    output_override = sys.argv[2] if len(sys.argv) > 2 else None

    with open(json_input_file) as f:
        j = json.load(f)

    # Components resolve relative `input`/`output` paths against this entry.
    j["json_input_file"] = json_input_file

    # Pin the output to an absolute path so every component writes into the
    # caller-provided directory regardless of whether it resolves `output`
    # relative to the cwd (e.g. isotropic_remeshing) or relative to the JSON
    # file's directory (e.g. image_simulation / topological_offset).
    if output_override is not None:
        j["output"] = output_override

    pywmtk.wmtk(j)
    return 0


if __name__ == "__main__":
    sys.exit(main())
