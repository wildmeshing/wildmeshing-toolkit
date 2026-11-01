# Developer guide

These pages are for changing the toolkit: fixing a component, adding an operation or a component,
or working on the core library. They describe the code as it is on `main`, with links to the
source.

- [The life of a component call](component-call.md): from `wmtk_app -j` or a Python call to the
  component's mesh and back.
- [The Wild driver](wild-driver.md): the optimization loop that tetwild, triwild, simwild and
  topological_offset share.
- [Writing an operation](writing-an-operation.md): the hooks, invariants and rollback, on the
  shortest edge collapse component.
- [Writing a component](writing-a-component.md): the checklist.
- [Testing](testing.md): what the test suites check and how CI runs them.
- [Writing documentation](documentation.md): this site, its examples and its generated reference.
- [Contributing](contributing.md): formatting, pull requests, the data repository.

The design notes go deeper on single subjects: [component I/O](../in-memory-io.md),
[tracking](../tracking.md), and [what simwild adopted from tetwild and
triwild](../simwild-adopted-behaviours.md).

## The repository

| Directory | What it holds |
|---|---|
| [`src/wmtk/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk) | The library, CMake target `wmtk::toolkit`: the meshes, the scheduler, envelopes, I/O, histories. |
| [`components/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/components) | One directory per component, and the CMake functions that register them (`components/cmake/`). |
| [`app/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/app) | `wmtk_app`, the command line, and the headers CMake generates for it. |
| [`python/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/python) | The `wildmeshing` package: the bindings (`python/src`) and its pure-Python part (`python/wildmeshing`). |
| [`tests/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/tests) | The library's Catch2 tests, the integration test driver, the Python tests. |
| [`docs/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/docs) | This site: pages, examples (`docs/examples`), the reference generator (`docs/tools`). |
| [`cmake/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/cmake) | One recipe per dependency (`cmake/recipes/`, most of them CPM), and the data pin. |
| `data/` | The clone of the [data repository](https://github.com/wildmeshing/data2). Not part of this repository (`.gitignore`). |

The top-level [`CMakeLists.txt`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/CMakeLists.txt)
adds `src/wmtk`, then `app`, `python/src` (with `WMTK_PYBIND`, on by default), `tests` (with
`BUILD_TESTING`), and `components` last: registering a component links it into `wmtk_app`, the
Python module and the integration tests, which must exist by then.

## The library: `src/wmtk`

The core files, at the top of `src/wmtk`:

| File | What it is |
|---|---|
| [`TriMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriMesh.h), `TriMesh.cpp` | The triangle mesh: connectivity, `Tuple` navigation, the local operations (split, collapse, swap, smooth, face split) with their `_before` / `_after` hooks, rollback, locks, consolidation. |
| [`TetMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetMesh.h), `TetMesh*.cpp` | The same for tetrahedra; each operation's connectivity change has its own file (`TetMeshEdgeCollapsingConn.cpp`, `TetMeshSwapMeshConnectivity.cpp`, ...), and `TetMeshTriangleInsertionConn.cpp` / `TetMeshCutTable.cpp` insert triangles into a tet mesh. |
| [`TriOptimizerMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriOptimizerMesh.h), `TriOptimizerMesh*.cpp` | What triwild, simwild-2D and topological_offset-2D share: vertex, edge and face attributes, the split / collapse / swap / smooth passes and the optimization loop. See [the Wild driver](wild-driver.md). |
| [`TetOptimizerMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetOptimizerMesh.h), `TetOptimizerMesh*.cpp` | The 3D counterpart, for tetwild, simwild-3D and topological_offset-3D. |
| [`OptimizerParameters.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/OptimizerParameters.h) | The parameters of the optimizer meshes, with the measurements behind their defaults. |
| [`ExecutionScheduler.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/ExecutionScheduler.hpp) | `ExecutePass`: the priority queues, the locks, the serial (`kSeq`) and partitioned parallel (`kPartition`) policies. See [scheduling](../concepts/scheduling.md). |
| [`AttributeCollection.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/AttributeCollection.hpp) | Per-simplex attributes, with the thread-local record that rollback restores. |
| [`SlotPool.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/SlotPool.hpp) | The preallocated connectivity storage and the atomic counter that hands out fresh slots to parallel operations. |
| [`MeshData.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/MeshData.hpp) | `MeshData`, `MeshMap` and `ComponentResult`: the meshes the memory route takes and returns. |
| [`OperationDryRun.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/OperationDryRun.hpp) | The thread-local dry-run flag the scheduler's screening uses. |
| `RationalPositions.h`, `ExactPosition.hpp` | Vertex positions kept both as doubles and, while a vertex cannot be rounded, as rationals. |
| `SurfaceTagAttributes.h` | Whether a face (3D) or edge (2D) is tracked surface, and which bounding-box side it is on. |
| `Types.hpp` | The Eigen and rational types used throughout. |

And the subdirectories:

| Directory | What it holds |
|---|---|
| [`envelope/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/envelope) | `Envelope` (exact, through fast-envelope, or sampled), `BoundaryEnvelope` (a tube around an open boundary), union and intersection envelopes, `KNN`. |
| [`history/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/history) | The recorded histories of a run, their file format and replay. See [tracking](../tracking.md). |
| [`io/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/io) | `ComponentIO` (the file and memory routes of a component), mesh readers and writers (`read_mesh` / `write_mesh`), VTU writers. |
| [`optimization/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/optimization) | AMIPS and envelope energies and the vertex smoother, on polysolve. |
| [`simplex/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/simplex) | Simplices named by their vertices, and collections of them. |
| [`threading/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/threading) | The toolkit's own task pool, `parallel_for`, concurrent queues, vertex mutexes and `enumerable_thread_specific`. |
| [`utils/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/utils) | Everything else: exact predicates, Delaunay and the VolumeRemesher embedding, partitioning, sizing-field helpers, winding numbers, `run_pass` and the localized retry loop, logging. |

## Components, the app and the bindings

Each component lives in `components/<name>/wmtk/components/<name>/`: its entry points
`<name>.hpp` / `<name>.cpp`, its mesh class, its options (`<name>_spec.json`) and its tests
(`tests/`). [Writing a component](writing-a-component.md) lists what a new one needs.

`wmtk_app` ([`app/main.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/app/main.cpp))
and the Python module ([`python/src/pywildmeshing.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/python/src/pywildmeshing.cpp))
know the components only through headers that CMake writes into `app/` at configure time:
[the life of a component call](component-call.md) follows one call through them.

## Tests and data

`tests/` holds the library's unit tests (`wmtk_tests`), the integration test driver
(`integration_tests.cpp`) and the Python tests (`tests/python`); each component's own tests are in
its `tests/` directory. [Testing](testing.md) describes them all.

The meshes and the integration configs are in the [data repository](https://github.com/wildmeshing/data2).
When tests are enabled, the build clones it into `data/` (CMake's `ExternalProject`, target
`wmtk_data_download`), at the commit pinned by `GIT_TAG` in
[`cmake/wmtk_data.cmake`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/cmake/wmtk_data.cmake).
Tests read it through the `WMTK_DATA_DIR` definition of the `wmtk::data` target. A build with
`BUILD_TESTING=OFF`, as the Python package's, does not clone it.

## Where to start reading

1. [`ShortestEdgeCollapse.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/shortest_edge_collapse/wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.cpp):
   the smallest complete use of a mesh, an operation and the scheduler, walked through in
   [writing an operation](writing-an-operation.md).
2. `TriMesh.h`, for `Tuple` and the hooks, and `TriMesh::collapse_edge` in `TriMesh.cpp`, for the
   order in which an operation calls them.
3. `ExecutePass` in `ExecutionScheduler.hpp`: its members are the callbacks a pass sets.
4. `TriOptimizerMesh::mesh_improvement` in `TriOptimizerMesh.cpp`, then
   [the Wild driver](wild-driver.md), before touching tetwild, triwild, simwild or
   topological_offset.
