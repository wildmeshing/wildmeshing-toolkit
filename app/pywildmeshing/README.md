# WMTK Python Bindings

`wildmeshing` provides Python bindings for WildMeshingToolkit through a single module entry point, `wildmeshing.wildmeshing(...)`.

## Installation

The recommended way to build and install the bindings is through `pip`, which uses `scikit-build-core` to drive CMake:

```bash
pip install .
```

For local development, an editable install is also supported:

```bash
pip install -e . -v
```

Where:

- `-e` (editable mode) installs the package in development mode so that changes to the source code are immediately reflected without reinstalling
- `-v` (verbose) shows detailed CMake and build output, useful for debugging build issues

The Python package metadata in `pyproject.toml` already enables the binding build and installs only the `python_bindings` component.

## Usage

Import the module, build a Python dictionary, and pass it to `wildmeshing.wildmeshing`:

```python
from wildmeshing import *

j = {
    "application": "tetwild",
    "input": ["input.obj"],
    "output": "output",
}

wildmeshing(j)
```

The `application` key is required. The wrapper checks that key, looks it up in the generated component map, and dispatches to the matching application. If `application` is missing or unknown, the call fails with an error.
