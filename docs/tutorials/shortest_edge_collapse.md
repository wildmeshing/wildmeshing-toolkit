# Shortest edge collapse

`shortest_edge_collapse` simplifies a triangle mesh to a target number of vertices by collapsing
edges ([Hoppe 1996](https://hhoppe.com/pm.pdf)), the shortest first. A collapse merges the two
ends of an edge at its midpoint, or onto one of them when that one has to stay where it is (a
vertex of a frozen boundary). The length of the edge is the only criterion: the collapses do not
look at the shape. With an envelope, no collapse may move the surface farther than a given
distance from the input.

Use it for a fast reduction of a dense mesh, where the shape matters less than the time; TetWild
simplifies its input surface with the same collapses. To keep the shape at a vertex budget, use
[QSlim](qslim.md), which collapses the edges of least quadric error first; for uniform,
well-shaped triangles, use [isotropic remeshing](isotropic_remeshing.md).

## A first run

The Octocat (18,944 vertices) down to a tenth of its vertices, inside an envelope of 0.2% of its
bounding box diagonal, the same run as the [QSlim tutorial](qslim.md#a-first-run):

=== "Command line"

    ```json
    --8<-- "docs/examples/json/shortest_edge_collapse.json"
    ```

    ```bash
    wmtk_app -j shortest_edge_collapse.json
    ```

    The input is read relative to the file; the simplified mesh, `octocat_sec.obj`, is written to
    the working directory.

=== "Python"

    ```python
    --8<-- "docs/examples/python/shortest_edge_collapse.py:memory"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/shortest_edge_collapse.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/shortest_edge_collapse.cpp:run"
    ```

    Link `wmtk::shortest_edge_collapse`.

It prints `18944 -> 1894 vertices`, or 1893: see [threads](#threads-and-reproducibility).

<div>
<wmtk-viewer example="shortest_edge_collapse"></wmtk-viewer>
</div>

The input on the left, the result on the right: drag to rotate, scroll to zoom. Every example is in the [gallery](../gallery/index.md).

## Key settings

### The target: `target_rel`

[`target_rel`](../reference/shortest_edge_collapse.md#target_rel) (default 0.1) is the target
number of vertices as a fraction of the input's: of the vertices its triangles use, after reading.
The bunny's file has 35,947 vertices and its triangles use 34,834, so `target_rel` 0.1 stops at
3,483. There is no absolute target. The collapses stop when the mesh reaches the target, or when
no collapse is left that the envelope and the link condition allow, whichever comes first.

### The envelope: `eps_rel`

[`eps_rel`](../reference/shortest_edge_collapse.md#eps_rel) is the envelope's thickness, relative
to the input's bounding box diagonal. The default, -1, is no envelope: the collapses go all the
way to the target, wherever that takes the surface. With an envelope, a collapse that would leave
part of the surface outside it is refused, and the shortest edges left may all be refused before
the target. On the Octocat, with `target_rel` 0.01 (189 vertices):

```python
--8<-- "docs/examples/python/shortest_edge_collapse.py:sweep"
```

| `eps_rel` | vertices left |
|---|---|
| -1 (none) | 186 |
| 0.01 | 391 |
| 0.002 | 1,856 |

With an envelope of 0.2%, it barely gets past the first run's tenth. Inside the same envelopes,
QSlim stops at roughly two thirds of these counts (see [its table](qslim.md#the-envelope-eps_rel)):
its collapses follow the shape, so fewer of them leave the envelope.

### The envelope test: `use_sample_envelope`

The envelope test is exact by default, and it is most of the run time: in the sweep above, a run
with an envelope takes close to a hundred times longer than the run without.
[`use_sample_envelope`](../reference/shortest_edge_collapse.md#use_sample_envelope) tests
triangles against the same envelope by sampling them, which is tens of times faster on this sweep.
To stay conservative between its samples, the sampled test refuses some collapses that keep the
surface inside, so it stops earlier:

```python
--8<-- "docs/examples/python/shortest_edge_collapse.py:sampled"
```

| `eps_rel` | exact | sampled |
|---|---|---|
| 0.01 | 391 | 501 |
| 0.002 | 1,856 | 2,194 |

Use it when the exact test is too slow: on large or pathological inputs, where one triangle covers
millions of tiny ones (some Thingi10K models).

### The open boundary: `boundary_eps_rel`

By default every vertex of an open boundary is frozen: two frozen vertices are never merged, and
a collapse between a boundary vertex and an interior one goes onto the boundary vertex. The
outline keeps every vertex it has. A positive
[`boundary_eps_rel`](../reference/shortest_edge_collapse.md#boundary_eps_rel) replaces the
freezing with a tube around the input's boundary: collapses may move the boundary, as long as
every boundary edge stays in the tube. The bunny has five holes, 223 boundary edges in all; here
without a surface envelope, to keep the run short:

```python
--8<-- "docs/examples/python/shortest_edge_collapse.py:boundary"
```

Both runs reach the target (3,483 vertices). Frozen, the boundary keeps its 223 edges; in a tube
of 0.2% it has 157. Set the tube to about `eps_rel` when the outline may be simplified with the
rest.

### The link condition: `use_link_condition`

[`use_link_condition`](../reference/shortest_edge_collapse.md#use_link_condition) (default true)
refuses a collapse that would change the topology: the result has the genus and the boundary
loops of the input, and no new non-manifold edges or vertices. Turned off, those collapses are
allowed, and the mesh can be simplified a little further, at the cost of its topology. On the
Octocat, with `target_rel` 0.01 and `eps_rel` 0.01:

```python
--8<-- "docs/examples/python/shortest_edge_collapse.py:link"
```

It stops at 378 vertices instead of 391 (379 and 393 sequentially), but the closed surface now has
22 boundary edges and 2 non-manifold edges. Keep the link condition on unless the topology does not matter, or is wrong
in the input already.

### Threads and reproducibility

The examples set [`num_threads`](../reference/shortest_edge_collapse.md#num_threads) to 4: on the
first run, that is about four times faster than one thread. With more than one thread, the order
of the collapses depends on the schedule, and the result changes slightly from run to run: 1,893
or 1,894 vertices on the first run, 391 to 402 at `eps_rel` 0.01 in the sweep. `num_threads` 0
runs sequentially and gives identical results.

## Outputs

- The simplified mesh: `output` on the command line (default `out.obj`), `outputs["mesh"]` in
  memory.
- In memory, the report holds the number of `vertices` and `triangles` of the result. There is no
  report file.
- With [`throw_on_fail`](../reference/shortest_edge_collapse.md#throw_on_fail), the run fails when
  the target is not reached, instead of writing the mesh it got to.
- With [`history_output`](../reference/shortest_edge_collapse.md#history_output), the history of
  the collapses, from which [tracking](tracking.md) maps points between the input and the output.

## Pitfalls

- **A misspelled option is ignored.** `shortest_edge_collapse` runs with the default of an option
  it does not know; `wmtk_app` and the Python bindings log a warning
  (`shortest_edge_collapse has no option ...`). Editors catch it beforehand with the
  [JSON Schemas](../json-schemas.md).
- **Duplicate vertices.** The input is welded first: vertices closer than
  [`remove_duplicate_eps`](../reference/shortest_edge_collapse.md#remove_duplicate_eps) (1e-5 of
  the diagonal) merge, so a mesh written triangle by triangle, as STL is, simplifies as one
  surface. A negative value keeps every vertex; then the triangles of an STL file stay apart,
  every vertex on a boundary and frozen, and nothing collapses.
- **The target is not reached.** The envelope refuses the remaining collapses: thicken it, accept
  the larger mesh, or use QSlim, which gets further inside the same envelope.
- **The target is passed without the link condition.** The run stops after a number of collapses,
  and a collapse that changes the topology can remove more than one vertex: on the Octocat without
  an envelope, `target_rel` 0.01 with `use_link_condition` false leaves fewer than 189 vertices.
- **The output is not next to the JSON file.** `wmtk_app` reads `input` relative to the JSON file,
  but writes `output` relative to the working directory.

## See also

- [Reference: every option of `shortest_edge_collapse`](../reference/shortest_edge_collapse.md)
- [QSlim](qslim.md), [isotropic remeshing](isotropic_remeshing.md)
- Hoppe 1996, [Progressive Meshes](https://hhoppe.com/pm.pdf)
