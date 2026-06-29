"""Subprocess helper: run a single integration-test JSON through the wildmeshing binding.

This mirrors what the C++ side does in both ``tests/integration_tests.cpp``
(``load_json`` + ``wmtk_wrapper``) and ``app/main.cpp``:

    1. parse the JSON input file,
    2. inject ``input_dir`` so components can resolve relative paths,

It is executed as its own process so that each test gets an isolated working
directory and global state, and so a crash in the native code fails only the
one parametrized case instead of the whole pytest session.

Usage:
    python _run_binding.py <input_dir>
"""

import json
import sys

from wildmeshing import *


def main() -> int:
    input_dir = sys.argv[1]

    with open(input_dir) as f:
        j = json.load(f)

    # Components resolve relative `input`/`output` paths against this entry.
    j["input_dir"] = input_dir

    wildmeshing(j)
    return 0


if __name__ == "__main__":
    sys.exit(main())
