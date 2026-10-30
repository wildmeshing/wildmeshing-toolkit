# Concepts

The Wildmeshing Toolkit describes mesh generation and mesh editing algorithms declaratively. An
algorithm is not a loop that walks the mesh and rewires it; it is a specification in four parts,
and the toolkit runs it:

- **Per-element invariants.** Conditions every element must satisfy after every change: no
  inverted triangle or tetrahedron, a surface that stays within a distance of the input, a link
  condition that keeps the topology. The toolkit checks them after each operation.
- **Operation scheduling.** Which local operations to try (split, collapse, swap, smooth), on
  which elements, in which order, and what to try next after one succeeds.
- **Attribute updates.** How the data stored on vertices, edges, faces and cells (positions,
  sizing, tags) is computed after an operation, written as hooks that run before and after it.
- **Rollback.** An operation whose hook or invariant fails is undone: the toolkit restores the
  connectivity and the attributes it changed, so the algorithm never sees an invalid mesh.

From this specification the toolkit runs the algorithm in parallel on shared memory. Operations
whose neighbourhoods do not overlap run at the same time; the code that describes the algorithm
contains no threads and no locks.

The abstraction is the one of the paper Jiang et al.,
[Declarative Specification for Unstructured Mesh Editing Algorithms](https://doi.org/10.1145/3550454.3555513),
ACM Transactions on Graphics 41(6), SIGGRAPH Asia 2022. The API has changed since; the version
the paper describes is the tag
[`1.0`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/1.0).

## The pages

| Page | What it explains |
|---|---|
| [Local operations](local-operations.md) | The mesh (connectivity plus attributes), tuples and navigation, the operations in 2D and 3D, hooks, invariants and rollback |
| [Scheduling](scheduling.md) | How a pass orders and runs operations, how it runs them in parallel, and when the result is reproducible |
| [Robust geometry](robust-geometry.md) | Exact (rational) positions and when they become doubles, exact predicates, the envelope and its two tests |
| [Quality](quality.md) | The AMIPS energy, smoothing, the stopping target, refinement when the optimization stalls, the coarsening pass |
| [Embedding](embedding.md) | How tetwild, triwild and simwild insert their input exactly into a background mesh, and how the outside is filtered away |
| [Regions](regions.md) | Per-cell tags, the expression language that selects them, and the components that use it |
| [Histories](histories.md) | What `history_output` records, and the map the tracking component rebuilds from it |

## Where the concepts live in the code

| Concept | Code |
|---|---|
| Triangle and tetrahedral meshes, operations, hooks, rollback | [`src/wmtk/TriMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriMesh.h), [`src/wmtk/TetMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetMesh.h) |
| Attributes and their rollback | [`src/wmtk/AttributeCollection.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/AttributeCollection.hpp) |
| The scheduler | [`src/wmtk/ExecutionScheduler.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/ExecutionScheduler.hpp) |
| The optimizer shared by tetwild, triwild, simwild and topological_offset | [`src/wmtk/TriOptimizerMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriOptimizerMesh.h), [`src/wmtk/TetOptimizerMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetOptimizerMesh.h), [`src/wmtk/OptimizerParameters.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/OptimizerParameters.h) |
| Envelopes | [`src/wmtk/envelope/`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/envelope/Envelope.hpp) |
| Smoothing and energies | [`src/wmtk/optimization/`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/optimization/SmoothVertex.hpp) |
| Histories | [`src/wmtk/history/`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/history/History.hpp) |
| Threads | [`src/wmtk/threading/`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/threading/Concurrency.hpp) |

The components are applications of these concepts. `shortest_edge_collapse`, `qslim` and
`isotropic_remeshing` derive their mesh from `wmtk::TriMesh` and define their own hooks and
passes. `tetwild`, `triwild`, `simwild` and `topological_offset` derive from the shared optimizer
bases `TetOptimizerMesh` and `TriOptimizerMesh`, which implement the split, collapse, swap and
smoothing passes once for all of them (see the [Wild driver](../developer/wild-driver.md)). To
use a component, start from the [tutorials](../tutorials/index.md); to write an operation or a
component, from the [developer guide](../developer/index.md).
