# Scheduling

An algorithm decides which operations to attempt and in which order; the toolkit runs them. A
*pass* takes a list of candidate operations, each a name and a tuple, and runs them through a
priority queue: it attempts the most urgent, queues the new candidates each success creates, and
stops when the queue is empty or a stopping criterion says so. The class that runs a pass is
`wmtk::ExecutePass<Mesh>`, in
[`src/wmtk/ExecutionScheduler.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/ExecutionScheduler.hpp).
The algorithm configures it with callbacks; how the operations are spread over threads is hidden
from it.

## A pass

Shortest edge collapse ([Hoppe 1996](https://hhoppe.com/pm.pdf)) is one pass: collapse every
edge, shortest first, until the mesh is down to a target number of vertices. Its candidates are
all the edges
([`ShortestEdgeCollapse.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/shortest_edge_collapse/wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.cpp)):

```cpp
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);
```

and the pass is configured and run like this (the `renew` callback returns an `edge_collapse` for
every edge of the triangles the collapse left around the surviving vertex):

```cpp
    auto measure_len2 = [](auto& m, auto op, const Tuple& new_e) {
        auto len2 =
            (m.vertex_attrs[new_e.vid(m)].pos - m.vertex_attrs[new_e.switch_vertex(m).vid(m)].pos)
                .squaredNorm();
        return -len2;
    };
    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_len2;
        executor.stopping_criterion_checking_frequency =
            target_vert_number > 0 ? (initial_size - target_vert_number - 1)
                                   : std::numeric_limits<int>::max();
        executor.stopping_criterion = [](auto& m) { return true; };
        executor(*this, collect_all_ops);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }
```

The operation itself (`collapse_edge`, with its hooks and invariants) is the
[local operation](local-operations.md); the pass only decides when to attempt it.

## The callbacks

| Member | What it decides |
|---|---|
| `edit_operation_maps` | The operations by name. The constructor registers `edge_split`, `edge_collapse`, `edge_swap`, `vertex_smooth` and `face_split`, and on a `TetMesh` also `face_swap`, `edge_swap_44`, `edge_swap_56` and `tet_split`. |
| `priority(m, op, t)` | A number per candidate; the largest is attempted first. Shortest edge collapse returns minus the squared length, so the shortest edge comes first. |
| `renew_neighbor_tuples(m, op, new_tuples)` | After a success, the new candidates, from the tuples the operation returned. |
| `should_renew(priority)` | Whether a renewed candidate is queued at all. |
| `is_weight_up_to_date(m, (priority, op, t))` | Whether a popped candidate still holds. A candidate is queued with the priority it had then; the mesh around it may have changed since, and this drops it if so. |
| `lock_vertices(m, t, task_id)` | Parallel passes only: claim the vertices the operation needs (see below). |
| `stopping_criterion(m)`, `stopping_criterion_checking_frequency` | When to stop before the queue is empty. |
| `on_fail(m, op, t)` | Called for a refused operation, for a driver that collects failures to retry. |

The loop pops the candidate with the largest priority, skips it if its tuple is no longer valid
or `is_weight_up_to_date` rejects it, attempts the operation, and on success queues what
`renew_neighbor_tuples` returns with their priorities. Ties are broken by the operation's name and
the tuple, so a serial pass pops candidates in the same order every time.

`stopping_criterion_checking_frequency` is a threshold, not a period: once the pass has had that
many successful operations, `stopping_criterion` is consulted after every operation. Shortest
edge collapse and QSlim set it to the number of collapses that reaches the target vertex count,
with a criterion that always says stop, so the pass ends on the collapse that reaches the target.
The passes of tetwild, triwild and simwild have no stopping criterion: each runs until its queue
is empty, and the optimization loop around them decides when to stop (see [Quality](quality.md)).

A pass that refuses an operation does not attempt it again. The optimizer passes run the
executor repeatedly until nothing succeeds, re-offering a failed operation only when a successful
one changed a vertex around it
([`LocalizedRetry.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/LocalizedRetry.hpp)).

## Parallel passes

The policy picks the execution: `ExecutionPolicy::kSeq` runs one queue on one thread;
`ExecutionPolicy::kPartition` runs the pass on `num_threads` threads. Every component chooses
`kSeq` when `num_threads` is 0 and `kPartition` otherwise.

**Partitions.** The mesh's vertices are divided into partitions. The optimizer meshes of tetwild,
triwild and simwild sort them by the Morton code of their positions and cut the order into
`num_threads` parts; with more than one thread, shortest edge collapse makes four partitions per
thread, so that a thread that drew light ones goes on to take more. Each candidate goes
to the queue of the partition of its tuple's vertex. Each thread takes a partition no other
thread has taken, drains its queue in priority order, and takes the next one. The priority order
therefore holds within a partition, not across the mesh.

**Locks.** Before an operation runs, its thread claims the vertices around the tuple with
`lock_vertices`: one mutex per vertex, acquired with a try-lock that never waits. A topology
change claims the two-ring of its edge, smoothing the one-ring of its vertex; an operation that
reads further asks for more (the [coarsening pass](quality.md#the-coarsening-pass) claims one ring
more than it smooths). Two operations whose claims overlap cannot run at the same time; all the
others can. An operation that loses the race for a lock is set aside and retried after other
work of the same thread; after `max_retry_limit` attempts (10) it goes to a queue that is drained
serially once every thread has finished.

**Screening.** Most attempted operations are refused by their first checks: on tetwild's hard
models about 98% are refused before they change anything. So a parallel pass runs in rounds, and
each round first *dry-runs* every candidate in parallel, without locks, against the unchanged
mesh: the before hook and the checks before the first change run, and the operation stops there.
Only the candidates that survive take locks and are attempted for real, with every check
repeated. Serial passes are not screened, nor is a pass with a stopping criterion; the 2D collapse
pass of the optimizer turns screening off, because it costs more than it saves there.

**Smoothing by colour.** In tetwild, triwild and simwild, parallel vertex smoothing uses no locks
by default ([`colored_smoothing`](../reference/tetwild.md#colored_smoothing)): the vertices are
divided into classes of pairwise non-adjacent vertices, and the classes are smoothed one after
another, each class entirely in parallel. Two vertices of a class share no cell, so neither
writes what the other reads.

**Storage.** The storage of vertices and cells cannot grow while threads run on it: it grows only
at serial points, between operations of a serial pass and between the rounds of a parallel one,
where each thread gets a budget of new slots for the next round.

The threads are the toolkit's own pool, built on `std::thread`
([`src/wmtk/threading/`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/threading/Concurrency.hpp)).
The workers are started once and reused from pass to pass.

## Threads and determinism

Every component that has a [`num_threads`](../reference/tetwild.md#num_threads) option defaults
it to 0.

- **`num_threads` 0** runs every pass serially, with a single queue that pops in a fixed order.
  The same input and options give the same output from run to run, on the same build and
  machine.
- **`num_threads` 1** runs the parallel scheduler on one thread: another code path (partitions,
  screening rounds), so another result than 0, though the same one from run to run.
- **`num_threads` 2 or more**: the order in which operations run depends on which thread wins
  which lock, and an operation that changes the mesh changes what the next ones see, so the
  result changes slightly from run to run. It also depends on the number of threads, which sets
  the partitions. Colour-class smoothing alone does not depend on the number of threads.

tetwild on `sphere.obj` from `data/models`, with default options, run twice per setting:

| `num_threads` | Output vertices, two runs | Identical outputs |
|---|---|---|
| 0 | 9,504 and 9,504 | yes |
| 1 | 9,511 and 9,511 | yes |
| 4 | 9,509 and 9,479 | no |

The [QSlim tutorial](../tutorials/qslim.md#threads-and-reproducibility) measures the speed-up
from 1 to 8 threads and the spread of the result on another model. For comparisons between
runs, for bug reports and for regression tests, use `num_threads` 0.

## See also

- [Local operations](local-operations.md): what a single operation does.
- [Writing an operation](../developer/writing-an-operation.md) and the
  [Wild driver](../developer/wild-driver.md) in the developer guide.
- Jiang et al. 2022, [Declarative Specification for Unstructured Mesh Editing Algorithms](https://doi.org/10.1145/3550454.3555513), for the design of the scheduler.
