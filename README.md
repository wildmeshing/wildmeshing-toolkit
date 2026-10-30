# Wildmeshing Toolkit

[![Build](https://github.com/wildmeshing/wildmeshing-toolkit/actions/workflows/continuous.yml/badge.svg)](https://github.com/wildmeshing/wildmeshing-toolkit/actions/workflows/continuous.yml)
[![Docs](https://github.com/wildmeshing/wildmeshing-toolkit/actions/workflows/docs.yml/badge.svg)](https://wildmeshing.github.io/wildmeshing-toolkit/)

Robust, parallel triangle and tetrahedral mesh generation and editing. Each algorithm is written
as a set of local operations (split, collapse, swap, smooth), the invariants every operation must
keep, and the rules that update attributes. The toolkit schedules the operations in parallel,
checks the invariants, and rolls back any operation that breaks them.

**Documentation, tutorials and examples: https://wildmeshing.github.io/wildmeshing-toolkit/**

[![The Octocat: its input surface, and the tet mesh TetWild makes of it, cut open and colored by element quality](https://wildmeshing.github.io/wildmeshing-toolkit/gallery/assets/tetwild_octocat/poster.png)](https://wildmeshing.github.io/wildmeshing-toolkit/gallery/)

## What is in this repository

| Directory | Contents |
|---|---|
| [`src/wmtk`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk) | The core library: triangle and tet mesh connectivity, local operations with rollback, the parallel scheduler, exact and sampled envelopes, rational positions, energies, mesh I/O, operation histories |
| [`components`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/components) | The algorithms built on the library, each configured by JSON options checked against its `*_spec.json` |
| [`app`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/app) | `wmtk_app`, the command line that runs any component from a JSON file |
| [`python`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/python) | The `wildmeshing` Python package: every component, on files or on NumPy arrays |
| [`tests`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/tests) | Unit, component and integration tests, in C++ and Python |
| [`docs`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/docs) | The documentation site, its tested examples, and design notes |

## Components

| Component | Input → output | What it does |
|---|---|---|
| [`shortest_edge_collapse`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/shortest_edge_collapse/) | triangle mesh → triangle mesh | Simplification by collapsing the shortest edges first ([Hoppe 1996](https://hhoppe.com/pm.pdf)) |
| [`qslim`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/qslim/) | triangle mesh → triangle mesh | Simplification by quadric error ([Garland and Heckbert 1997](https://dl.acm.org/doi/10.1145/258734.258849)) |
| [`isotropic_remeshing`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/isotropic_remeshing/) | triangle mesh → triangle mesh | Uniform remeshing to a target edge length, keeping feature curves ([Botsch and Kobbelt 2004](https://ls7-gv.cs.tu-dortmund.de/downloads/publications/2004/sgp04.pdf)) |
| [`tetwild`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/tetwild/) | triangle soups → tet mesh | Robust tetrahedral meshing of imperfect surfaces ([TetWild](https://cims.nyu.edu/gcl/papers/2018-TetWild.pdf)) |
| [`triwild`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/triwild/) | curve networks → triangle mesh | Robust 2D triangulation of curves ([TriWild](https://cims.nyu.edu/gcl/papers/2019-TriWild.pdf)) |
| [`simwild`](https://wildmeshing.github.io/wildmeshing-toolkit/reference/simwild/) | surfaces or curves → multi-region mesh | Multi-region meshing, and region editing: remeshing, hole filling, sealing, overlap resolution, tag replacement |
| [`topological_offset`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/topological_offset/) | tagged tet mesh → tagged tet mesh | A region at a target distance around a selection ([Topological Offsets](https://dl.acm.org/doi/10.1145/3731157)); 2D input is refused for now |
| [`manifold_extraction`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/manifold_extraction/) | tagged mesh → tagged mesh | A manifold boundary for a selected region |
| [`tracking`](https://wildmeshing.github.io/wildmeshing-toolkit/tutorials/tracking/) | operation history → map | Maps points and fields between a remeshing's input and output |
| `prismatic_mesh` | offset tet mesh → prism/tet mesh | *Work in progress.* Thins the offset band around a solid to a target thickness and rebuilds it as prisms |
| `c1_simplification` | surface, tet and UV meshes → simplified meshes | *Work in progress.* Simplification within a deviation threshold, for C1 (Clough–Tocher) surfaces |

Every option of every component is in the
[reference](https://wildmeshing.github.io/wildmeshing-toolkit/reference/).

## Installation

Requirements: CMake 3.25 or newer, a C++17 compiler, and GMP (`apt install libgmp-dev`,
`brew install gmp`; on Windows CMake downloads it). CMake downloads every other dependency.

The command line, the C++ library and the tests:

```bash
git clone https://github.com/wildmeshing/wildmeshing-toolkit.git
cd wildmeshing-toolkit
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The Python package, from the repository root:

```bash
python -m pip install .
```

The `wildmeshing` package on PyPI today (0.4) is the older
[wildmeshing-python](https://github.com/wildmeshing/wildmeshing-python); install from source until
this one is released there. See [Installation](https://wildmeshing.github.io/wildmeshing-toolkit/getting-started/installation/)
for the options.

## Usage

From the command line, with a JSON file of options (input paths relative to the file, outputs
written to the working directory):

```bash
./build/app/wmtk_app --list                         # the components
./build/app/wmtk_app --options isotropic_remeshing  # one component's options
./build/app/wmtk_app -j data/integration_tests/isotropic_remeshing_bunny.json
```

```json
{
  "application": "isotropic_remeshing",
  "input": "../models/bunny.obj"
}
```

From Python, on files or on NumPy arrays:

```python
import wildmeshing as wm

# files in, files out
wm.tetwild({"input": ["bunny.obj"], "output": "bunny_tets"})

# arrays in, arrays out
outputs, report = wm.qslim({"input": "surface", "target_rel": 0.1},
                           {"surface": wm.read_mesh("bunny.obj")})
V = outputs["mesh"]["V"]
```

From C++, linking `wmtk::qslim`:

```cpp
#include <wmtk/components/qslim/qslim.hpp>
#include <wmtk/io/mesh_io.hpp>

wmtk::MeshMap meshes{{"surface", wmtk::io::read_mesh("bunny.obj")}};
auto [outputs, report] = wmtk::components::qslim::qslim(
    {{"input", "surface"}, {"target_rel", 0.1}}, meshes);
wmtk::io::write_mesh("bunny_simplified.vtu", outputs.at("mesh"));
```

[A first run](https://wildmeshing.github.io/wildmeshing-toolkit/getting-started/first-run/) walks
through the three.

## Contributing

See [CONTRIBUTING.md](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/CONTRIBUTING.md) and the
[developer guide](https://wildmeshing.github.io/wildmeshing-toolkit/developer/).

## Citing

The toolkit is described in *Declarative Specification for Unstructured Mesh Editing
Algorithms*, ACM Transactions on Graphics (SIGGRAPH Asia 2022); see
[Citing](https://wildmeshing.github.io/wildmeshing-toolkit/about/citing/) for the BibTeX and the
papers of each algorithm. The API has changed since; the version the paper describes is tag
[`1.0`](https://github.com/wildmeshing/wildmeshing-toolkit/releases/tag/1.0).

## License

[MIT](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/LICENSE). The libraries the
toolkit builds with have their own licenses.
