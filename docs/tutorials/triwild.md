# TriWild

`triwild` triangulates the plane around curve networks as they come: polylines that cross,
overlap, or miss each other by a little. It simplifies the curves inside an envelope, inserts them
exactly into a triangulation of their bounding box, so that every input segment becomes a chain of
triangle edges and every crossing a vertex, and then improves the triangles' shape while the
inserted curves stay within a distance \(\varepsilon\) of the input
([Hu et al. 2019](https://cims.nyu.edu/gcl/papers/2019-TriWild.pdf)). A filter can remove what lies
outside the curves at the end.

Use it to mesh a 2D domain bounded by drawn or traced curves, or several overlapping regions at
once: each triangle records which inputs it lies inside. For the same in 3D, from triangle surfaces,
use [TetWild](tetwild.md); for regions in a mesh you already have, [SimWild](simwild.md).

## A first run

Five closed curves that overlap, each its own input file, named after its shape:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/triwild.json"
    ```

    ```bash
    wmtk_app -j triwild.json
    ```

    The input paths are relative to the JSON file; the outputs, the triangle mesh `shapes.msh`
    among them (see [outputs](#outputs)), are written to the working directory.

=== "Python"

    ```python
    --8<-- "docs/examples/python/triwild.py:memory"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/triwild.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/triwild.cpp:run"
    ```

    Link `wmtk::triwild`.

The 719 input vertices (716 segments) are simplified to 297, the arrangement inserts them into a
triangulation of the bounding box, and the optimization ends with 265 to 289 vertices and 407 to
455 triangles over seven runs (see [threads](#threads-and-reproducibility)), the largest energy
between 6.1 and 8.8 for a target of 10. A run takes a fraction of a second. The Python and C++
examples then print, for each name, the number of triangles inside that curve:

```text
blob1: 157 triangles
blob2: 143 triangles
circle: 62 triangles
ellipse: 19 triangles
rectangle: 110 triangles
```

The counts add up to more than the total: where the curves overlap, a triangle lies inside
several of them.

## Key settings

The numbers below come from the `sweep` section of the example, which changes one option at a
time against the first run (with `num_threads` 0, so that they repeat exactly):

```python
--8<-- "docs/examples/python/triwild.py:sweep"
```

### The inputs: `input`, `input_names`, `output`

[`input`](../reference/triwild.md#input) is a list of OBJ files of polylines (`v` and `l` lines),
up to 64. Each file is one input: its curves are kept apart from the others' in the output tags,
and each tracked edge is held to the envelope of the inputs it lies on. Vertices that coincide
exactly are merged when a file is read; a vertex of a curve file that no segment uses is kept as a
[feature point](#feature-points-input_points-preserve_feature_points).
[`input_names`](../reference/triwild.md#input_names) gives the inputs names, in the same order:
they label the tag columns of the output (`tag_0`, `tag_1`, ... without them).
[`output`](../reference/triwild.md#output) is the file name without extension.

In memory, `input` names entries of the meshes dictionary, and the input names can be the same
strings.

### The envelope: `eps_rel`

[`eps_rel`](../reference/triwild.md#eps_rel) (default 0.001) is the distance the output's curves
may be from the input, relative to the bounding box diagonal. The input is simplified within half
of it, so detail smaller than \(\varepsilon\) may disappear, and the optimization keeps every
tracked edge within \(\varepsilon\) of the input. A thinner envelope keeps more of the input's vertices
and constrains the optimization more:

| `eps_rel` | \(\varepsilon\) | triangles | largest distance to the input | iterations |
|---|---|---|---|---|
| 0.0002 | 0.025 | 977 | 0.018 | 10 |
| 0.001 (default) | 0.127 | 428 | 0.090 | 2 |
| 0.005 | 0.637 | 297 | 0.447 | 1 |

The distance is the containment that `DEBUG_hausdorff` measures: the largest distance from a
point of the output's curves to the input. It stays below \(\varepsilon\) in every run.

### The edge length: `length_rel`

[`length_rel`](../reference/triwild.md#length_rel) (default 0.05) is the target edge length,
relative to the bounding box diagonal. Edges longer than 4/3 of it are split, and edges shorter
than 4/5 of it are collapsed when the envelope and the quality allow. Away from the curves it sets
the size of the triangles; along them, the curves' own detail and the envelope do:

| `length_rel` | triangles | max energy |
|---|---|---|
| 0.05 (default) | 428 | 6.85 |
| 0.02 | 1,549 | 7.18 |
| 0.01 | 6,185 | 5.00 |

### The quality target: `stop_energy`

The shape of a triangle is measured by its AMIPS energy: 2 for an equilateral triangle, growing
without bound as the triangle flattens (see [Quality](../concepts/quality.md)). The optimization
stops when the largest energy is below [`stop_energy`](../reference/triwild.md#stop_energy), or
after [`max_iterations`](../reference/triwild.md#max_iterations) (80). The default, 100, is a loose
target in 2D, 50 times the minimum; the example asks for 10:

| `stop_energy` | triangles | max energy | average | iterations |
|---|---|---|---|---|
| 100 (default) | 423 | 21.47 | 3.74 | 1 |
| 10 | 428 | 6.85 | 2.91 | 2 |
| 5 | 498 | 4.36 | 2.58 | 8 |
| 3 | 680 | 2.95 | 2.26 | 6 |

The average barely moves until the target gets close to the minimum: what the target buys is the
worst triangles. When the largest energy stops improving, the triangles above the target are
refined; with a target close to 2 that is nearly all of them, so set
[`stuck_refine_num_worst`](../reference/triwild.md#stuck_refine_num_worst) to a positive number
there.

### The outside: `filter`

The triangulation covers a box around the input.
[`filter`](../reference/triwild.md#filter) decides what is kept (see [Embedding](../concepts/embedding.md) for how each works):

| `filter` | Keeps | On the five shapes |
|---|---|---|
| `none` (default) | everything | 924 triangles |
| `flood` | all but the region the curves separate from the box's border | 457 triangles |
| `input` | the triangles inside at least one input, by each input's winding number | 428 triangles |
| `tracked` | the triangles inside the tracked curves, oriented as the input was | 79 triangles |

`flood` keeps 29 triangles more than `input`: they lie in pockets enclosed by the union of the
curves but inside none of them. `tracked` counts all the curves together, with their orientation:
four of the shapes run clockwise and the rectangle counterclockwise, so the winding number is −1
inside a clockwise shape and 0 where one overlaps the rectangle, and only the part of the
rectangle that no other curve covers is kept. `input` orients each input on its own. Use it for
several inputs. All three need closed curves (see the [pitfalls](#pitfalls)).

### Feature points: `input_points`, `preserve_feature_points`

[`input_points`](../reference/triwild.md#input_points) lists files whose vertices are inserted as
vertices of the triangulation (their edges are ignored). The endpoints of open polylines are kept
the same way. [`preserve_feature_points`](../reference/triwild.md#preserve_feature_points) (on by
default) anchors them: some vertex stays within \(\varepsilon\) of each, and a collapse that would
remove it is refused. Here a closed curve and twelve points, nine inside it and three outside:

```python
--8<-- "docs/examples/python/triwild.py:points"
```

| `preserve_feature_points` | points with a vertex within \(\varepsilon\) | farthest point from a vertex |
|---|---|---|
| true (default) | 12 of 12 | 0 |
| false | 0 of 12 | 1.50, 44 \(\varepsilon\) |

Without the anchors, the collapses take the points' vertices like any other. The points outside
the curve's bounding box grow it, and with it \(\varepsilon\) and the edge length, which are
relative to its diagonal. A point in a region the filter discards leaves the mesh with that region,
but stays in the feature output.

### Topology: `preserve_topology`

By default the simplification and the optimization may change the curves' topology: close a gap
narrower than \(\varepsilon\), merge two junctions, remove a loop smaller than \(\varepsilon\).
[`DEBUG_euler`](../reference/triwild.md#DEBUG_euler) logs the Euler characteristic of each
connected curve at each stage, which shows such changes.
[`preserve_topology`](../reference/triwild.md#preserve_topology) forbids it: only exactly
coincident vertices are merged, the simplification does not merge junctions or endpoints, and the
operations check the link condition. Set it when the curves' connectivity matters, after making
sure the input has no gaps far below \(\varepsilon\) (see the [pitfalls](#pitfalls)).

### Threads and reproducibility

[`num_threads`](../reference/triwild.md#num_threads) 0 runs sequentially and gives the same mesh
every time (428 triangles here). With threads, the order of the operations depends on the
schedule, and the result changes slightly from run to run: seven runs of the first run gave 407
to 455 triangles. On inputs of this size a run takes a fraction of a second either way.

## Outputs

On files, `output` names:

- `shapes.msh`: the triangles, with the vertices in 3D (z = 0). Each triangle has its `quality`
  (its energy) and `tags`: one column per input, labelled by `input_names`, 1 where the triangle is
  inside that input. When the filter keeps triangles that are inside no input, an `ambient` column
  marks them.
- `shapes.vtu`: the same triangles for ParaView, with a field `tag_<i>` per input, `quality`, and
  `flood_fill` (the regions the curves separate) when the flood fill ran
  ([`compute_flood_fill`](../reference/triwild.md#compute_flood_fill)).
- `shapes_surf.vtu`: the tracked curves, the edges of the mesh that lie on the input.
- `shapes_features.obj`: with feature points only, the tracked curves (`l`) and the anchored
  points (`p`), collected before the filter.
- With [`report`](../reference/triwild.md#report), a JSON file with the vertex and triangle counts
  (`#v`, `#t`), `max_energy`, `avg_energy`, `#iterations`, `eps`, the coarsening statistics, and,
  with `DEBUG_hausdorff`, the distances `hausdorff` (output to input, the containment) and
  `coverage` (input to output); with `DEBUG_feature_retention`, `features_retained` and
  `features_total`. A value of -1 means not measured.

In memory, the outputs are `mesh` (as in the `.msh`), `surface` (the tracked curves, as edges),
`features` (with feature points) and `envelope` (the input curves the envelope is built around).
The report holds the same keys as the file.

[`history_output`](../reference/triwild.md#history_output) records every change to the tracked
curves, from the input files to the output, and [tracking](tracking.md) maps points between the
two from it. Recording anchors the junctions too, which changes the output, unless
[`allow_junction_cleanup`](../reference/triwild.md#allow_junction_cleanup) is given.

## Pitfalls

- **Open curves have no inside.** The filters decide what is inside from the curves, and an open
  polyline does not bound anything. On `triwild_polylines.obj`, a drawing of 46 polylines with 94
  loose ends:

    ```python
    --8<-- "docs/examples/python/triwild.py:open"
    ```

    `flood` keeps the inside of the drawing's frame (1.35 million square units of the box's 2.76),
    while `input` and `tracked` keep 1.60 million, a band of triangles above the frame included.
    On open curves, use `filter` `none` and select the triangles from the output yourself.
- **Several inputs with `filter` `tracked`.** It sums all curves with their orientation; inputs
  that run the other way cancel (79 triangles of 428 above). Use `input`.
- **`preserve_topology` and tiny gaps.** The rectangle of the first run is two polylines whose
  corners miss each other by \(3 \cdot 10^{-15}\). By default a collapse closes the gap during
  the optimization. With `preserve_topology` it stays, and the optimization cannot get rid of the
  near-degenerate triangles there: on the rectangle alone, the largest energy is
  \(7.8 \cdot 10^{8}\) after 30 iterations, and on the five shapes it does not converge in 80.
  With the coordinates rounded to 9 digits, the same run converges in one iteration. Weld such
  vertices in the input before asking for the topology to be kept: under `preserve_topology`,
  [`remove_duplicate_eps`](../reference/triwild.md#remove_duplicate_eps) is ignored.
- **A misspelled option is ignored.** `wmtk_app` and the Python bindings log a warning
  (`triwild has no option ...`) and run with the defaults. Editors catch it beforehand with the
  [JSON Schemas](../json-schemas.md).

## See also

- [Reference: every option of `triwild`](../reference/triwild.md)
- [Embedding](../concepts/embedding.md): the simplification, the arrangement and the filters;
  [Quality](../concepts/quality.md): the energy and the optimization loop.
- [TetWild](tetwild.md), the same in 3D; [SimWild](simwild.md) for regions in a mesh;
  [tracking](tracking.md) to map points through the history.
- Hu et al. 2019, [TriWild: Robust Triangulation with Curve Constraints](https://cims.nyu.edu/gcl/papers/2019-TriWild.pdf)
