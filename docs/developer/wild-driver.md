# The Wild driver

tetwild, triwild, simwild and topological_offset improve their meshes with the same loop of
splits, collapses, swaps and smoothing, written once per dimension:
[`TetOptimizerMesh`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetOptimizerMesh.h)
for tet meshes and
[`TriOptimizerMesh`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriOptimizerMesh.h)
for triangle meshes. Each component derives its mesh from one of them, inherits the passes and
the loop, and changes what it needs through virtual hooks. This page describes the loop as the
code runs it; the measurements behind the defaults are in the comments of
[`OptimizerParameters.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/OptimizerParameters.h).

## Who uses it

| Component | Mesh class | Base | What runs the loop |
|---|---|---|---|
| tetwild | `TetWildMesh` | `TetOptimizerMesh` | `mesh_improvement(max_iterations)`, after the input is simplified and embedded; `use_legacy_code` runs the older `mesh_improvement_legacy` instead |
| triwild | `TriWildMesh` | `TriOptimizerMesh` | `mesh_improvement(max_iterations)` |
| simwild | `SimWildMesh` (3D), `SimWildMeshTri` (2D) | `TetOptimizerMesh`, `TriOptimizerMesh` | `mesh_improvement(max_iterations)`, for the operation `remeshing` |
| topological_offset | `TopoOffsetTetMesh`, `TopoOffsetTriMesh` | `TetOptimizerMesh`, `TriOptimizerMesh` | its own turns of `local_operations()` while it places the offset front, then `mesh_improvement()` for the final quality pass |

The two bases are deliberately separate: the same algorithms on different primitives, with a
different quality convention (a tet stores AMIPS cubed and the 3D loop reports its cube root; a
triangle stores the 2D AMIPS energy). The header comment of each base says why it was seeded by
moving tetwild's (3D) and triwild's (2D) implementation into it.

## The loop: `mesh_improvement()`

`TetOptimizerMesh::mesh_improvement` in
[`TetOptimizerMesh.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetOptimizerMesh.cpp),
with logging and debug output left out; the 2D loop in `TriOptimizerMesh.cpp` is the same,
without `optimization_iteration_done()`:

```cpp
compute_vertex_partition_morton();

if (optimization_bare_coarsen_passes()) {
    // Performing swaps after the initial collapse improves quality and convergence.
    local_operations({{0, 1, 1, 0}}, false);
}

double pre_max_metric = std::get<0>(optimization_quality_stats());
int refine_cooldown = m_params.stuck_refine_cooldown;

for (int it = 0; it < max_its; ++it) {
    if (!m_params.interleaved_smoothing) {
        std::tie(max_metric, avg_metric) =
            local_operations({{1, 1, 1, m_params.num_smoothing_passes}});
    } else {
        const int k = m_params.interleaved_smoothing_passes;
        const std::array<std::array<int, 4>, 3> passes = {
            {{{1, 0, 0, k}}, {{0, 1, 0, k}}, {{0, 0, 1, k}}}};
        for (const auto& ops : passes) {
            std::tie(max_metric, avg_metric) = local_operations(ops);
            if (max_metric < optimization_stop_metric()) break;
        }
    }
    optimization_iteration_done(it);

    // count the rounded vertices: cnt_round of cnt_verts
    if (max_metric < optimization_stop_metric()) {
        if (cnt_round == cnt_verts) break;
    }

    consolidate_mesh();

    if (optimization_stop_at_float() && round_and_check_all_rounded()) {
        break;
    }

    if (refine_cooldown > 0) {
        --refine_cooldown;
    } else if (
        it > 0 && max_metric > optimization_stop_metric() &&
        optimization_stalled(pre_max_metric, max_metric)) {
        refine_sizing_around_worst(max_metric);
        refine_cooldown = m_params.stuck_refine_cooldown;
    }
    pre_max_metric = max_metric;
}

if (optimization_bare_coarsen_passes()) {
    local_operations({{0, 1, 1, 0}});
}

coarsen_mesh();
```

**The partition.** At the start, the vertices are cut into `NUM_THREADS` parts along a Morton
order of their positions; a parallel pass queues every operation in the part of its tuple's
vertex (see [scheduling](../concepts/scheduling.md)). With `num_threads` 0 there is no
partition, and every pass is serial.

**The bare passes.** Before the loop, a collapse pass without the length gate
(`collapse_limit_length` false), then a swap pass: they remove what the embedding left. After
the loop, a collapse pass with the gate, then a swap pass. When
`optimization_bare_coarsen_passes()` returns false, neither runs. topological_offset's 2D mesh
returns false; its 3D mesh returns true only under TetWild's rules, in its final pass. The offset
surface is what its loop places, no envelope holds it, and an ungated collapse pass would
decimate it (`TetOptimizerMesh.h` has the measurement).

**An iteration.** With [`interleaved_smoothing`](../reference/tetwild.md#interleaved_smoothing)
(on by default in all four components), three groups: splits, collapses, swaps, each followed by
`interleaved_smoothing_passes` smoothing passes, and the iteration ends early after a group that
brings the metric under the stop. Without it, one group: splits, collapses and swaps, then
`num_smoothing_passes` smoothing passes.

**The stop.** The loop ends when the max metric is under `optimization_stop_metric()` and every
vertex is rounded to doubles, or after `max_iterations`. For tetwild and triwild the metric is the
max AMIPS energy and the stop is [`stop_energy`](../reference/tetwild.md#stop_energy). simwild
overrides both: its metric is each cell's quality relative to its region's target, and the stop
is 1. With simwild's `stop_at_float`, `optimization_stop_at_float()` also ends the loop as soon as
every vertex is rounded, and skips it when every vertex already is.

**Consolidation.** After every iteration that does not stop, `consolidate_mesh()` compacts the
storage: ids change, and every tuple held across it is invalid.

**Stuck refinement.** When an iteration improved the max metric by at most
`stuck_refine_stall_eps` times the distance still to cover, the mesh is stalled:

```cpp
return (prev - cur) <= m_params.stuck_refine_stall_eps * (cur - optimization_stop_metric());
```

`refine_sizing_around_worst()`, which each application implements, then lowers the sizing field
around the worst cells. tetwild's ([`TetWildMesh.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/tetwild/wmtk/components/tetwild/TetWildMesh.cpp))
picks the cells above a filter energy, marks the longest edge of each for a forced split in the
next split pass (`stuck_refine_force_split`), multiplies the sizing scalar of their vertices
(grown by `stuck_refine_rings` rings) by `stuck_refine_factor`, down to `stuck_refine_min_scalar`,
and grades the result into its surroundings (`stuck_refine_gradation`). After a refinement, the
next `stuck_refine_cooldown` iterations do not refine.

**Coarsening.** `coarsen_mesh()` returns at once unless
[`coarsen_pass`](../reference/tetwild.md#coarsen_pass) is set; it is not gated by the bare passes.
Then, for at most `coarsen_max_rounds` rounds, it runs a collapse pass in coarsening mode, where a
collapse is kept only if the worst cell in the region it touched is no worse than before, then
`coarsen_global_smoothing_passes` smoothing passes; it stops after a round that collapses nothing.

## A group: `local_operations()`

`local_operations(ops)` takes four counts, `{split, collapse, swap, smooth}`, and runs, in this
order:

| Count | What runs |
|---|---|
| `ops[0]` | `split_all_edges()`, that many times |
| `ops[1]` | `collapse_all_edges(collapse_limit_length)`, that many times |
| `ops[2]` | the swaps, at most that many times, stopping after a pass that swaps nothing: `swap_all_edges()` in 2D, `swap_all_edges_all()` then `swap_all_faces()` in 3D |
| `ops[3]` | `smooth_all_vertices(ops[3])`, then `round_all_vertices()` |

After each kind that ran it logs the max and average energy, runs the sanity checks when
`DEBUG_sanity_checks` is set, and calls `update_attributes()` (in 3D,
`update_attributes_and_history()`). If a pass ran out of preallocated
slots (`slots_exhausted()`), it consolidates, which reclaims the slots of removed simplices and
re-derives the storage from the live count, and runs the same kind again.

## The passes

The split, collapse and swap passes collect their candidates in parallel
(`parallel_collect_edge_ops`, `parallel_collect_face_ops`). Every pass runs them through an
`ExecutePass` built by `run_pass`
([`src/wmtk/utils/RunPass.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/RunPass.hpp)):
`kPartition` with `NUM_THREADS > 0`, `kSeq` otherwise, with the lock of its kind. All but
smoothing run through `run_localized_to_convergence`
([`LocalizedRetry.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/LocalizedRetry.hpp)):
the executor runs again until a round succeeds nowhere, and a failed operation is retried only
if one of its vertices was in the region a successful operation renewed during that round.

| Pass | Candidates | Popped first | Dropped from the queue when | Lock |
|---|---|---|---|---|
| split | every edge, `edge_split` | the longest | its length changed since it was queued, or it is not due: shorter than 4/3 of the target length times the sizing (`splitting_l2`), and not a forced split | edge two-ring |
| collapse | every edge, both ways, `edge_collapse` | the shortest | its length changed, or, with the length gate, it is longer than 4/5 of the target length times the sizing (`collapsing_l2`) | edge two-ring |
| swap, 2D | every edge, `edge_swap` | the largest drop in max energy (`swap_weight`) | its drop changed, or is under 1e-5 | edge two-ring |
| swap, 3D | every edge, as `edge_swap`, `edge_swap_44` and `edge_swap_56`; then every face, `face_swap` | the longest edge | | edge two-ring; face two-ring for face swaps |
| smoothing | every vertex; with `skip_good_regions`, only the vertices of cells whose energy is at least `skip_good_regions_margin` times the stop energy, and every tracked-surface vertex | | | vertex one-ring, or none (colored) |

The target length is [`length_rel`](../reference/tetwild.md#length_rel) times the bounding box
diagonal (`OptimizerParameters::init_lengths_from_diagonal`); the sizing is the mean of the two
endpoints' sizing scalars, which stuck refinement lowers.

Three details matter when changing a pass:

- **Longest-edge order.** In a parallel split pass an edge that loses a lock race is set aside,
  and the thread moves on to shorter edges. `must_wait` makes a split wait while a longer due
  edge of one of its cells is still queued, so that the parallel pass splits in the order the
  serial one does (the comment in `TriOptimizerMeshSplit.cpp` has the measurements).
- **Screening.** A parallel pass first dry-runs every candidate without locks, and only the
  survivors take their ring locks ([writing an operation](writing-an-operation.md#dry-runs)).
  The 2D collapse pass turns it off (`executor.screen_before_commit = false`), where it costs
  more than it saves.
- **Colored smoothing.** With `colored_smoothing` (the default, except in topological_offset) and
  more than one thread, smoothing does not lock: the vertices are split into classes of pairwise
  non-adjacent vertices (`utils::greedy_vertex_coloring`), and each class runs in parallel. A
  smoothing hook may then write only its vertex and its own cells, and must read everything
  outside the vertex's star through const access; every smooth checks it and throws on the first
  offence. In 3D, `check_surface_folds` turns colored smoothing off.

## What an application overrides

The bases implement the operation hooks (`split_edge_before`, `collapse_edge_after`, ...) and
call narrower virtual hooks from them; tetwild, triwild and simwild override only those.
topological_offset also overrides the operation hooks themselves.

| Hook | Default | Overridden by |
|---|---|---|
| `refine_sizing_around_worst(max)` | none (pure virtual) | every application |
| `optimization_quality_stats()`, `optimization_stop_metric()` | max and average AMIPS; `stop_energy` | simwild (relative quality, stop at 1) |
| `optimization_stop_at_float()` | false | simwild (`stop_at_float`) |
| `optimization_bare_coarsen_passes()` | true | topological_offset |
| `optimization_stalled(prev, cur)` | the stall test above | none |
| `collapse_quality_allowed` | allowed when the removed vertex is not rounded, or the new cells are no worse than the ring they replace (in 2D also: under `stop_energy`) | simwild, topological_offset |
| `swap_quality_allowed` | strict improvement of the max energy | topological_offset |
| `collapse_before_vertex`, `collapse_after_vertex`, `split_after_vertex`, ... | accept, do nothing | tetwild (features, open boundaries), simwild, topological_offset |
| `smoothing_energy_envelope(vid)` | 2D: the mesh's envelope for a surface vertex; 3D: none (pure virtual) | tetwild, triwild, simwild-3D, topological_offset |
| `cell_quality(tid)`, `set_cell_quality(tid, q)` (3D) | none (pure virtual): the cell attributes are the application's | tetwild, simwild, topological_offset |
| `allow_surface_swap()`, `check_surface_topology()` (3D) | none (pure virtual) | tetwild and simwild (their options of the same names), topological_offset |

The headers list the rest, each with what it may and may not do.

## Changing the shared code

A change to `TriOptimizerMesh` or `TetOptimizerMesh` changes four components at once, and
tetwild and simwild also simplify their input with
[`ShortestEdgeCollapse`](writing-an-operation.md). Run the unit tests of all four
(`wmtk_test_tetwild`, `wmtk_test_triwild`, `wmtk_test_simwild`, `wmtk_test_topological_offset`)
and the integration configs in a Release build ([testing](testing.md)). When simwild moved onto
the bases, the rule was that tetwild's and triwild's output must not move where the copies
disagreed, and simwild's could; [simwild: behaviours adopted from tetwild /
triwild](../simwild-adopted-behaviours.md) records each place where it did.
