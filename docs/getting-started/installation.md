# Installation

The toolkit is a C++ library with a command line, `wmtk_app`, and a Python package,
`wildmeshing`. Both are built from source.

## Requirements

- CMake 3.25 or newer, and a C++17 compiler. CI builds with GCC on Linux (x86-64 and arm64),
  Apple Clang on macOS, and Visual Studio 2026 on Windows.
- [GMP](https://gmplib.org/):

    === "Linux"

        ```bash
        sudo apt install libgmp-dev
        ```

    === "macOS"

        ```bash
        brew install gmp
        ```

    === "Windows"

        Nothing to do: CMake downloads a build of GMP when it does not find one.

Every other dependency is downloaded by CMake on the first configure.

## The command line and the C++ library

```bash
git clone https://github.com/wildmeshing/wildmeshing-toolkit.git
cd wildmeshing-toolkit
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This builds the library, every component, `build/app/wmtk_app`, the Python module (in
`build/bin`) and the tests. Build in Release: the components run whole meshing pipelines, and
Debug builds are 10 to 100 times slower.

Options worth knowing (`-D<option>=ON|OFF`):

| Option | Default | What it does |
|---|---|---|
| `BUILD_TESTING` | `ON` | Build the tests, and clone the [example data](https://github.com/wildmeshing/data2) into `data/` |
| `WMTK_PYBIND` | `ON` | Build the Python module |
| `WMTK_BUILD_DOC_EXAMPLES` | `OFF` | Build this documentation's C++ examples and test them with the Python and JSON ones |
| `WMTK_ENABLE_COMPONENT_<name>` | `ON` | Build one component; turn off the ones you do not need |
| `WMTK_BUILD_DOCS` | `OFF` | The Doxygen C++ reference (target `wmtk_doc_doxygen`) |

The examples on this site read their meshes from `data/models`, which the configure step clones
when `BUILD_TESTING` is on (the default).

Check the build:

```bash
./build/app/wmtk_app --list
```

lists the components and what each does; `--options <name>` lists a component's options.

## The Python package

From the repository root:

```bash
python -m pip install .
```

This builds the bindings in Release (in `build/Release`, a few minutes the first time) and
installs the `wildmeshing` package; it does not build the tests or download the example data.
With the PolyFEM-based operations' dependencies:

```bash
python -m pip install ".[polyfem]"
```

Check it:

```python
import wildmeshing as wm
print(wm.components())
help(wm.qslim)
```

!!! note "PyPI"
    The package will be published on PyPI as `wildmeshing`. The `wildmeshing` package on PyPI
    today (0.4) is the older [wildmeshing-python](https://github.com/wildmeshing/wildmeshing-python)
    bindings of TetWild and TriWild, with a different interface: install from source until the
    new release is out.

### Faster rebuilds with uv

When you change the toolkit and rebuild the package often:

```bash
uv venv --python 3.14 .venv
uv pip install --python .venv/bin/python numpy scikit-build-core pybind11
CMAKE_BUILD_PARALLEL_LEVEL=8 uv pip install --python .venv/bin/python --no-build-isolation .
```

Without build isolation, the second install rebuilds only what changed. To reuse the sources
CMake already downloaded for another build of the toolkit, add
`-C cmake.define.CPM_SOURCE_CACHE=<that build>/CPM`.

### From a CMake build

The CMake build above also builds the module, into `build/bin/wildmeshing`:

```bash
export PYTHONPATH=$PWD/build/bin
python -c "import wildmeshing"
```

The module is built for the Python CMake finds; use the same interpreter to import it.

## Using the library from your own CMake project

Added to another project (`add_subdirectory`, `FetchContent` or CPM), the toolkit builds its core
library, `wmtk::toolkit`, and not the components: those are built only when the toolkit is the
top-level project. To call a component from your own C++ code today, build your program in the
toolkit's tree, as the [documentation's examples](../developer/documentation.md#examples) are
(`docs/examples/CMakeLists.txt`), linking `wmtk::<component>`.

## Next

[A first run](first-run.md): the same remeshing from the command line, Python and C++.
