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

From PyPI, with wheels for macOS arm64, Linux x86_64 and arm64, and Windows x86_64:

```bash
python -m pip install wildmeshing
```

Versions before 2.0.0 on PyPI are the older
[wildmeshing-python](https://github.com/wildmeshing/wildmeshing-python) bindings of TetWild and
TriWild, with a different interface.

From source, from the repository root:

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

### Faster rebuilds with uv

When you change the toolkit and rebuild the package often:

```bash
uv venv --python 3.14 .venv
uv pip install --python .venv/bin/python numpy scikit-build-core pybind11 pybind11-stubgen==3.0.0
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
library, `wmtk::toolkit`. With `WMTK_BUILD_COMPONENTS` on, it also builds the components, and your
targets link `wmtk::<component>` and call it as the
[documentation's C++ examples](../developer/documentation.md#examples) do:

```cmake
set(WMTK_BUILD_COMPONENTS ON CACHE BOOL "" FORCE)
add_subdirectory(wildmeshing-toolkit)   # or FetchContent / CPMAddPackage
target_link_libraries(my_program PRIVATE wmtk::triwild wmtk::tracking)
```

As a subproject the toolkit builds no command line, no Python module (`WMTK_PYBIND` is off unless
set), and none of its tests or test data, whatever your project's `BUILD_TESTING`. Inside its own
scope it uses its own CMake recipes (`cmake/recipes`) before any of yours with the same names,
default symbol visibility and C++17 unless a target asks for more, so a project built with
`-fvisibility=hidden` or C++20 links it as is.

## Next

[A first run](first-run.md): the same remeshing from the command line, Python and C++.
