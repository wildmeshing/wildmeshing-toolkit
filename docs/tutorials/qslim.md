# QSlim

`qslim` simplifies a triangle mesh to a target number of vertices. It collapses the edges of
least quadric error first ([Garland and Heckbert 1997](https://dl.acm.org/doi/10.1145/258734.258849)),
each to the point that minimizes the error, so flat regions lose their vertices first and sharp
features keep theirs. With an envelope, no collapse may move the surface farther than a given
distance from the input.

Use it to reduce a mesh to a vertex budget while keeping its shape. For a coarse mesh with
uniform, well-shaped triangles, use [isotropic remeshing](isotropic_remeshing.md) instead;
[shortest edge collapse](shortest_edge_collapse.md) is the simpler criterion, faster but blind to
the shape.

## A first run

The Octocat (18,944 vertices) down to a tenth of its vertices, inside an envelope of 0.2% of its
bounding box diagonal:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/qslim.json"
    ```

    ```bash
    wmtk_app -j qslim.json
    ```

    The input path is relative to the JSON file; the output, `octocat_qslim.obj`, is written to
    the working directory.

=== "Python"

    ```python
    --8<-- "docs/examples/python/qslim.py:memory"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/qslim.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/qslim.cpp:run"
    ```

    Link `wmtk::qslim`.

It prints `18944 -> 1891 vertices`, give or take a few: see
[threads](#threads-and-reproducibility).

<div>
<wmtk-viewer example="qslim"></wmtk-viewer>
</div>

The input on the left, the result on the right: drag to rotate, scroll to zoom. Every example is in the [gallery](../gallery/index.md).

## Key settings

### The target: `target_rel`, `target_abs`

[`target_rel`](../reference/qslim.md#target_rel) (default 0.1) is the target number of vertices as
a fraction of the input's; [`target_abs`](../reference/qslim.md#target_abs), when positive,
replaces it with an absolute count. The collapses stop when the mesh reaches the target, or when
no collapse is left that the envelope allows, whichever comes first.

### The envelope: `eps_rel`

[`eps_rel`](../reference/qslim.md#eps_rel) is the envelope's thickness, relative to the input's
bounding box diagonal. The default, -1, is no envelope: the collapses go all the way to the
target, wherever that takes the surface. With an envelope, a collapse that would leave part of
the surface outside it is refused, and a thin envelope stops the simplification before the
target. On the Octocat, with `target_rel` 0.01 (189 vertices):

```python
--8<-- "docs/examples/python/qslim.py:sweep"
```

| `eps_rel` | vertices left |
|---|---|
| -1 (none) | 188 |
| 0.01 | 267 |
| 0.002 | 1,139 |

So the envelope is what decides how far a simplification can go while the result still
approximates the input; the target is an upper bound.

### The open boundary: `boundary_eps_rel`

On a mesh with an open boundary, the surface envelope alone holds the boundary loosely: the
outline can retract along the surface. A positive
[`boundary_eps_rel`](../reference/qslim.md#boundary_eps_rel) adds a tube around the input's
boundary that every boundary edge must stay in, and places collapses at the boundary on the
outline. Set it to about `eps_rel` when the outline matters.

### The envelope test: `use_sample_envelope`

The envelope test is exact by default. [`use_sample_envelope`](../reference/qslim.md#use_sample_envelope)
switches to a sampled test, which is much faster on pathological inputs, where one triangle
covers millions of tiny ones (some Thingi10K models). The envelope is the same; the test is
approximate.

### Threads and reproducibility

The envelope test is most of the time: on the Octocat with `eps_rel` 0.002, one thread takes
11.1 s, 8 threads 1.5 s. Without an envelope the same run takes 0.5 s. The examples set
[`num_threads`](../reference/qslim.md#num_threads) to 4. With more than one thread, the order of
the collapses depends on the schedule, and the result changes slightly from run to run (1,887 to
1,894 vertices here); `num_threads` 0 runs sequentially and gives identical results.

## Outputs

- The simplified mesh: `output` on the command line (default `out.obj`), `outputs["mesh"]` in
  memory.
- With `report`, a JSON file with the target, the vertex and triangle counts and the run time.
- With `throw_on_fail`, the run fails when the target is not reached instead of writing the mesh
  it got to.
- With `history_output`, the history of the collapses, from which [tracking](tracking.md) maps
  points between the input and the output.

## Pitfalls

- **A misspelled option is ignored.** `qslim` runs with the default of an option it does not
  know; `wmtk_app` and the Python bindings log a warning (`qslim has no option ...`). Editors
  catch it beforehand with the [JSON Schemas](../json-schemas.md).
- **The target is not reached.** The envelope refuses the remaining collapses: thicken it, or
  accept the larger mesh. `throw_on_fail` turns this into an error.
- **Duplicate vertices.** The input is welded first (vertices within 1e-5 of the diagonal are
  merged), so a mesh written face by face simplifies as one surface.

## See also

- [Reference: every option of `qslim`](../reference/qslim.md)
- [Shortest edge collapse](shortest_edge_collapse.md), [isotropic remeshing](isotropic_remeshing.md)
- Garland and Heckbert 1997, [Surface Simplification Using Quadric Error Metrics](https://dl.acm.org/doi/10.1145/258734.258849)
