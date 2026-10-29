"""Python bindings for the Wildmeshing Toolkit.

Every component is a function taking its JSON options: `name(options)` runs on
files, `name(options, meshes)` runs in memory on named mesh dictionaries and
returns `(outputs, report)`. `read_mesh` / `write_mesh` convert between files and
mesh dictionaries; `save` / `load` write and read memory results as checkpoints.

The PolyFEM-backed operations are in `wildmeshing.polyfem_ops` (install the
`polyfem` extras).
"""
from ._core import *  # noqa: F401,F403
from .checkpoint import load, save  # noqa: F401
