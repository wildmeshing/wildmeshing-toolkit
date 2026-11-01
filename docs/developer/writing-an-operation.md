# Writing an operation

The local operations themselves (split, collapse, swap, smoothing, ...) are written once, in
`TriMesh` and `TetMesh`. An application customizes them by deriving a mesh class and overriding
hooks: a `_before` and an `_after` per operation, and `invariants`. It then runs them with an
`ExecutePass`, which decides the order and the parallelism. This page describes what the mesh
calls and when, what a failed operation undoes, and then walks through the shortest edge
collapse component, the smallest complete example. [Local operations](../concepts/local-operations.md)
describes the operations themselves, and [scheduling](../concepts/scheduling.md) the scheduler.

## The order of the calls

`TriMesh::collapse_edge`
([`src/wmtk/TriMesh.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriMesh.cpp)),
with the debug checks and the bookkeeping left out:

```cpp
bool TriMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_tris)
{
    if (!collapse_edge_before(loc0)) {
        return false;
    }
    // ... refuse a collapse that would leave the surviving vertex without a face
    if (dry_run()) return true; // checks done, the change would come next

    collapse_edge_conn(loc0, new_tris, return_t, new_vid, old_tris, old_vertices, ...);

    start_protect_attributes();

    if (!collapse_edge_after(return_t) || !invariants(new_tris)) {
        collapse_edge_rollback(new_vid, old_tris, old_vertices, ...);
        rollback_protected_attributes();
        return false;
    }
    // ... commit the edit to the history, if one is recorded
    release_protect_attributes();
    return true;
}
```

Every operation of `TriMesh` and `TetMesh` has this shape:

1. **`<op>_before(t)`** runs on the mesh as it is. It decides whether the operation is worth
   trying, and saves what `_after` will need: positions, flags, the energy before. Returning
   false ends the operation; nothing has changed.
2. The mesh's own checks: a `TriMesh` swap refuses a boundary edge, a collapse one that would
   leave the surviving vertex without a face. The `TriMesh` link condition is the base
   `collapse_edge_before` itself (unless `set_use_link_condition(false)`): an override that wants
   it calls `TriMesh::collapse_edge_before(t)` first, as shortest edge collapse does.
3. In a dry run the operation stops here and returns true ([below](#dry-runs)).
4. The connectivity changes, and the operation keeps what it needs to undo it. Some checks come
   with the change and can still refuse here: a `TetMesh` collapse tests the link condition and
   duplicate tets as it rewires them.
5. Attribute protection starts: from here on, attribute writes are recorded.
6. **`<op>_after(t)`** runs on the new connectivity. It sets the attributes of what the operation
   created or changed (the position of the new vertex, the quality of the new cells) and may
   still refuse: false rolls the operation back.
7. **`invariants(cells)`** runs on the cells around the change; false rolls the operation back.
8. On success the edit goes into the history, if one is recorded, and the attribute record is
   dropped.

The tuple `_after` receives depends on the operation. A `TriMesh` collapse keeps the second
vertex of the edge (`t.switch_vertex(m)`) and removes the first, and `collapse_edge_after` gets a
tuple at the surviving vertex. `split_edge_after` gets the half of the old edge at its first
vertex: `t.switch_vertex(m)` is the new vertex. The comments in `TriMesh.h` and `TetMesh.h`
describe the others.

## Invariants

`invariants(const std::vector<Tuple>& cells)` is called after every operation's `_after`, with
the cells it created or changed: in a `TriMesh`, the triangles around the surviving vertex of a
collapse, the one-ring of the new vertex of a split, the triangles of a swap, the one-ring of a
smoothed vertex. It is the place for what every operation must keep (the surface inside its
envelope, no inverted element), so that a new operation added to the class keeps it too. The
default returns true.

## Thread-local caches

`_before` and `_after` are two calls, and in a parallel pass other threads run other operations
in between on the same mesh object. What `_before` saves for `_after` therefore goes into a
per-thread value, never a plain member:

```cpp
struct PositionInfoCache
{
    Eigen::Vector3d v1p;
    Eigen::Vector3d v2p;
    // ...
};
wmtk::threading::enumerable_thread_specific<PositionInfoCache> position_cache;
```

`position_cache.local()` returns the calling thread's value, created on first use. An operation
runs on one thread from `_before` to the end, so what that thread wrote in `_before` is what it
reads in `_after`.
[`enumerable_thread_specific`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/threading/enumerable_thread_specific.hpp)
is the toolkit's own (it replaced TBB's); it is neither copyable nor movable, and so neither is a
mesh class that holds one.

## What rollback restores

When `_after` or `invariants` says no, the operation is undone:

- **The connectivity** of every cell and vertex the operation changed, from the copies the
  connectivity step kept.
- **The attributes** this thread reached through the non-const `operator[]` of a registered
  `AttributeCollection`, since `start_protect_attributes()`: each entry gets back the value it had
  at its first access. A collection is registered by pointing the mesh's
  `p_vertex_attrs`, `p_edge_attrs`, `p_face_attrs` (and, in a `TetMesh`, `p_tet_attrs`) at it,
  usually in the constructor; an `AttributeContainerGroup` registers several for one simplex type.

It does not restore:

- what `_before` wrote to the mesh's attributes: protection starts after it. `_before` writes
  to its cache only.
- members of the mesh class, counters, caches, and collections not registered.
- the slots a split took: they stay used, marked removed, until the next `consolidate_mesh()`.

Reading through the non-const `operator[]` also records the entry. Read with `.at(i)` or through
a const reference: it copies nothing, and it keeps the record to what the operation wrote, which
matters where the record is checked (colored smoothing throws if a smoothing hook wrote outside
its vertex's star, see [the Wild driver](wild-driver.md#the-passes)).

Set every field of a vertex the operation creates rather than relying on what its slot holds,
as `IsotropicRemeshing::split_edge_after` does (`va = VertexAttributes{}`, then the fields).

## Locks

In a parallel pass, the scheduler locks a region around the operation's tuple before calling it
and releases it after: for the operations queued on edges, the vertices of the two-ring of the
edge (`try_set_edge_mutex_two_ring`), for smoothing the one-ring of the vertex. The hooks may
read and write only inside that region. An operation that reaches farther (one that smooths a
ring of vertices inside a collapse, say) needs a lock of a larger radius:
`try_set_edge_mutex_n_ring(e, task_id, n)`, which `run_pass`
([`src/wmtk/utils/RunPass.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/RunPass.hpp))
uses when it is given a radius.

## Dry runs

A parallel `ExecutePass` with `screen_before_commit` (on by default) runs in rounds, and each
round first **dry-runs** every candidate, in parallel and without locks
([`src/wmtk/OperationDryRun.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/OperationDryRun.hpp)):
the operation runs up to its first change and returns true. Only the candidates that pass are
then tried for real, with their locks, every check repeated. So in a dry run `_before` runs
concurrently with other `_before` calls on an unchanging mesh, without a lock, and may write only
thread-local caches and atomic counters. A hook that has to write the mesh tests
`dry_run()` and skips the write, answering as if it had succeeded. A counter in `_before` sees a
candidate twice when it passes the dry run.

Serial passes and passes with a stopping criterion (shortest edge collapse and qslim with a
target) are not screened.

## Running it: `ExecutePass`

`ExecutePass<AppMesh>`
([`src/wmtk/ExecutionScheduler.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/ExecutionScheduler.hpp))
takes a list of `(operation name, tuple)` pairs and runs them in priority order. Its members are
the rules of the pass:

| Member | What it decides | Default |
|---|---|---|
| `edit_operation_maps` | the operation each name runs | `edge_split`, `edge_collapse`, `edge_swap`, `vertex_smooth`, `face_split`; in a `TetMesh` also `edge_swap_44`, `edge_swap_56`, `face_swap`, `tet_split` |
| `priority(m, op, t)` | the order: the largest value is popped first | 0 |
| `is_weight_up_to_date(m, elem)` | whether a popped entry still holds (its priority may be stale) | true |
| `renew_neighbor_tuples(m, op, new_cells)` | what a success puts back in the queue | nothing |
| `should_renew(priority)` | whether a renewed entry is queued | true |
| `lock_vertices(m, t, task)` | the region a parallel operation locks; false if another thread holds part of it | always succeeds |
| `stopping_criterion`, `stopping_criterion_checking_frequency` | when the pass stops early | never |
| `on_fail(m, op, t)` | what to do with a failed operation | nothing |
| `num_threads`, the policy (`kSeq`, `kPartition`) | serial, or one queue per partition of the mesh | `kSeq` |

An entry whose cell has changed since it was queued is dropped before it runs: a `Tuple` stores
the hash its cell had when the tuple was made, an operation on the cell changes the cell's hash,
and `is_valid` compares the two. With `kPartition` the mesh class must provide
`get_partition_id(const Tuple&)`.

## Worked example: shortest edge collapse

[`ShortestEdgeCollapse`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/shortest_edge_collapse/wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h)
simplifies a triangle mesh by collapsing its shortest edge first ([Hoppe 1996](https://hhoppe.com/pm.pdf)),
until it reaches a vertex count, optionally keeping the surface inside an envelope. It is one
operation, the edge collapse, three hooks and one pass. tetwild and simwild also use it, to
simplify their input surfaces, so a change here changes them too.

### The data

```cpp
struct VertexAttributes
{
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    bool freeze = false;
    bool input_boundary = false;
};

class ShortestEdgeCollapse : public wmtk::TriMesh
{
public:
    wmtk::Envelope m_envelope;
    bool m_has_envelope = false;
    wmtk::BoundaryEnvelope m_boundary_envelope;
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;
    wmtk::AttributeCollection<FaceAttributes> face_attrs;
    // ...
};
```

The constructor registers both collections, so that rollback and consolidation reach them:

```cpp
NUM_THREADS = (num_threads);
// ...
p_vertex_attrs = &vertex_attrs;
p_face_attrs = &face_attrs;
```

`create_mesh()` builds the connectivity (`TriMesh::init`), the envelope when `eps > 0`, and the
partition (`partition_mesh()`: four parts per thread when there is more than one), and then
holds the open boundary: with a positive `boundary_eps` a `BoundaryEnvelope`, a tube around the
input's boundary edges; otherwise `freeze_boundary()` freezes every boundary vertex.

### `collapse_edge_before`

```cpp
bool ShortestEdgeCollapse::collapse_edge_before(const Tuple& t)
{
    if (!TriMesh::collapse_edge_before(t)) return false;

    // v1 is removed by the collapse, v2 survives (TriMesh::collapse_edge_conn keeps vid2).
    const size_t v1 = t.vid(*this);
    const size_t v2 = t.switch_vertex(*this).vid(*this);
    auto& cache = position_cache.local();
    cache.v1_frozen = vertex_attrs[v1].freeze;
    cache.v2_frozen = vertex_attrs[v2].freeze;

    if (cache.v1_frozen && cache.v2_frozen) {
        return false;
    }

    cache.v1_input_boundary = vertex_attrs[v1].input_boundary;
    cache.v2_input_boundary = vertex_attrs[v2].input_boundary;
    cache.v1_on_boundary = wmtk::BoundaryEnvelope::on_input_boundary(*this, vertex_attrs, t);
    cache.v2_on_boundary =
        wmtk::BoundaryEnvelope::on_input_boundary(*this, vertex_attrs, t.switch_vertex(*this));

    cache.v1p = vertex_attrs[v1].pos;
    cache.v2p = vertex_attrs[v2].pos;
    return true;
}
```

It first asks the base for the link condition (the component's `use_link_condition`). Two frozen
endpoints cannot merge without moving one of them, so it refuses those. Everything else it only
reads, into the thread-local cache: the endpoints' positions and flags, read before the
connectivity changes.

### `collapse_edge_after`

```cpp
bool ShortestEdgeCollapse::collapse_edge_after(const TriMesh::Tuple& t)
{
    const auto& cache = position_cache.local();
    const Eigen::Vector3d p = cache.v1_frozen                                 ? cache.v1p
                              : cache.v2_frozen                               ? cache.v2p
                              : cache.v1_on_boundary && !cache.v2_on_boundary ? cache.v1p
                              : cache.v2_on_boundary && !cache.v1_on_boundary
                                  ? cache.v2p
                                  : (cache.v1p + cache.v2p) / 2.0;
    const size_t vid = t.vid(*this);
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].freeze = cache.v1_frozen || cache.v2_frozen;
    vertex_attrs[vid].input_boundary = cache.v1_input_boundary || cache.v2_input_boundary;

    return true;
}
```

`t` is at the surviving vertex. It goes to the frozen endpoint if one is frozen, to the boundary
endpoint if exactly one is on the input's boundary, and to the midpoint otherwise; it takes over
the frozen and boundary flags. The writes come after `start_protect_attributes()`, so a collapse
that `invariants` refuses gets its vertex back where it was.

### `invariants`

```cpp
bool ShortestEdgeCollapse::invariants(const std::vector<Tuple>& new_tris)
{
    // First: it touches only boundary edges, and is far cheaper than the surface test.
    if (!m_boundary_envelope.boundary_edges_inside(*this, vertex_attrs, new_tris)) {
        return false;
    }
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            bool outside = m_envelope.is_outside(tris);
            if (outside) return false;
        }
    }
    return true;
}
```

The triangles around the surviving vertex must keep their boundary edges in the boundary tube,
when there is one, and must lie inside the envelope, when there is one. The surface test reads
the positions through the non-const `operator[]`, so the neighbours' entries are recorded too:
harmless here, and what `.at()` would avoid.

### The pass: `collapse_shortest`

```cpp
size_t initial_size = get_vertices().size();
auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);

auto renew = [&](auto& m, auto op, auto& tris) {
    // ... progress logging, every 10 s
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, Tuple>>();
    for (auto& e : edges) optup.emplace_back("edge_collapse", e);
    return optup;
};
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

- **The candidates**: every edge once, as `edge_collapse`, the name of `TriMesh::collapse_edge`
  in `edit_operation_maps`.
- **The priority**: minus the squared length, so the shortest edge is popped first.
- **The renewal**: after a collapse, the edges of the triangles around the surviving vertex go
  back in the queue with their new lengths (`new_edges_after`). The entries queued before for
  those triangles fail `is_valid` and are dropped.
- **The stop**: the criterion returns true, so the pass stops the first time it is consulted,
  which is once the successes exceed `stopping_criterion_checking_frequency`: after
  `initial_size - target_vert_number` collapses, each of which removes one vertex (in a parallel
  pass, the operations already running finish). The comment on that member in
  `ExecutionScheduler.hpp` explains why it is a threshold and not a period. The live criterion
  also keeps the pass out of screening, so the collapses run in priority order up to the count.
- **The policy**: with threads, one queue per partition (`get_partition_id` returns the vertex's
  `partition_id`), each operation locking the two-ring of its edge; without, one serial queue.

The component around it
([`shortest_edge_collapse.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/shortest_edge_collapse/wmtk/components/shortest_edge_collapse/shortest_edge_collapse.cpp))
reads the options and the mesh, calls `create_mesh` and `collapse_shortest`, then
`consolidate_mesh()` and the output, as [the life of a component call](component-call.md)
describes.

### Trying a change

```bash
cmake --build build --target wmtk_app wmtk_test_shortest_edge_collapse
ctest --test-dir build -R wmtk_test_shortest_edge_collapse --output-on-failure
build/app/wmtk_app -j data/integration_tests/shortest_edge_collapse_octocat.json
```

The integration config sets `throw_on_fail`, so a run that misses the target fails. The
[tutorial](../tutorials/shortest_edge_collapse.md) shows the options from the user's side.
