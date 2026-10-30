# Isotropic remeshing

`isotropic_remeshing` remeshes a triangle mesh towards a uniform target edge length
([Botsch and Kobbelt 2004](https://ls7-gv.cs.tu-dortmund.de/downloads/publications/2004/sgp04.pdf)).
Each iteration splits the edges longer than 4/3 of the target, collapses those shorter than 4/5
of it, swaps edges to bring the valences closer to 6 (4 on the boundary), and moves every vertex
towards the average of its neighbours, within its tangent plane. Nothing holds the surface by
default: with an envelope, no operation may move it farther than a given distance from the input;
with features, curves and vertices the remeshing keeps.

Use it for uniform, well-shaped triangles at a chosen edge length, finer or coarser than the
input. To reduce a mesh to a vertex budget while keeping its shape, use [QSlim](qslim.md); for a
volume mesh, [TetWild](tetwild.md).

## A first run

The bunny (35,947 vertices in the file, 34,834 in its triangles) remeshed to edges of 1% of its
bounding box diagonal:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/isotropic_remeshing.json"
    ```

    ```bash
    wmtk_app -j isotropic_remeshing.json
    ```

    The input is read relative to the file; the remeshed mesh, `bunny_remeshed.obj`, and the
    report, `bunny_report.json`, are written to the working directory.

=== "Python"

    ```python
    --8<-- "docs/examples/python/isotropic_remeshing.py:memory"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/isotropic_remeshing.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/isotropic_remeshing.cpp:run"
    ```

    Link `wmtk::isotropic_remeshing`.

It prints `35947 -> 7929 vertices`. The run is sequential, so the result is the same every time.

## Key settings

### The target length: `length_rel`, `length_abs`

[`length_abs`](../reference/isotropic_remeshing.md#length_abs) is the target edge length;
when it is negative (the default), [`length_rel`](../reference/isotropic_remeshing.md#length_rel)
gives it relative to the bounding box diagonal; when both are negative, the target is the input's
average edge length. Since edges between 4/5 and 4/3 of the target are left alone, the average
edge of the result is somewhat longer than the target. On the bunny:

```python
--8<-- "docs/examples/python/isotropic_remeshing.py:length"
```

| `length_rel` | vertices | average edge / target |
|---|---|---|
| 0.005 | 30,313 | 1.18 |
| 0.01 | 7,929 | 1.15 |
| 0.02 | 2,034 | 1.12 |

Halving the length multiplies the vertex count by about four, as it should on a surface.

### The iterations: `max_iterations`

[`max_iterations`](../reference/isotropic_remeshing.md#max_iterations) (default 3) is the number
of iterations of splits, collapses, swaps and smoothing; all of them run. The first does most of
the work. On the bunny, the share of triangles with an angle under 30 degrees:

```python
--8<-- "docs/examples/python/isotropic_remeshing.py:iterations"
```

| | under 30 degrees |
|---|---|
| input | 6.77% |
| 1 iteration | 1.66% |
| 3 | 0.76% |
| 10 | 0.69% |

The smallest angle stays at 0.02 degrees: the worst triangles are along the frozen boundary, and
more iterations do not reach them (see [the boundary](#the-open-boundary-freeze_boundary)).

### The envelope: `eps_rel`

[`eps_rel`](../reference/isotropic_remeshing.md#eps_rel) is the envelope's thickness, relative to
the input's bounding box diagonal. The default, -1, is no envelope: smoothing moves every vertex
within its tangent plane, which on a curved surface or at a sharp edge is off the surface, and
the iterations add up. A box (`piece_0.obj`, 8 vertices), remeshed with `length_rel` 0.05, keeps
none of its corners: they round off. With an envelope, an operation that would leave part of the
surface outside it is refused:

```python
--8<-- "docs/examples/python/isotropic_remeshing.py:envelope"
```

| `eps_rel` | farthest vertex from the box |
|---|---|
| -1 (none) | 0.023 |
| 0.01 | 0.0066 |
| 0.001 | 0.00069 |

The diagonal is 1.22, so each envelope holds with room to spare. To keep sharp edges sharp, rather
than within a distance, make them [features](#features-features-feature_fields).

The envelope test is exact by default, and costs most of the run: on the bunny, an envelope of
0.2% makes the first run close to a hundred times slower, nearly all of it in the smoothing, and
the mesh stays somewhat finer where collapses are refused.
[`use_sample_envelope`](../reference/isotropic_remeshing.md#use_sample_envelope) tests against the
same envelope by sampling: there, about thirty times faster than the exact test, but it refuses
more operations and leaves more vertices. With more than one thread
([`num_threads`](../reference/isotropic_remeshing.md#num_threads)), the result changes slightly
from run to run (7,878 and 7,916 vertices for the first run with 4 threads, 7,929 sequentially);
0, the default, runs sequentially.

### The open boundary: `freeze_boundary`

By default ([`freeze_boundary`](../reference/isotropic_remeshing.md#freeze_boundary) true), the
vertices of an open boundary are frozen: they never move, and no collapse touches them. Splits
still refine the boundary edges. The bunny has 223 boundary edges, around five holes:

```python
--8<-- "docs/examples/python/isotropic_remeshing.py:boundary"
```

| | boundary edges | smallest angle |
|---|---|---|
| frozen (default) | 230 | 0.02 |
| `freeze_boundary` false | 102 | 1.94 |
| `boundary_eps_rel` 0.001 | 196 | 0.00 |

Frozen, the boundary keeps every input vertex, and the triangles next to it keep their short
edges: they are the thinnest of the result. With `freeze_boundary` false, the boundary is
remeshed like the rest, but nothing holds it: the holes can change shape. A positive
[`boundary_eps_rel`](../reference/isotropic_remeshing.md#boundary_eps_rel) remeshes the boundary
inside a tube around the input's boundary edges. In all three runs, the thinnest triangles are
along the boundary: the swaps by valence leave them there, and
[`swap_quality`](#the-swaps-swap_quality) removes most of them.

### Features: `features`, `feature_fields`

Features are curves and vertices the remeshing keeps. A feature edge is never swapped; a split
leaves two feature edges; a vertex on a curve moves only along it, and collapses only along it.
The ends of a curve, the vertices where three or more curves meet or where the edge id changes,
and fixed vertices stay where they are. They come in two forms:

- [`features`](../reference/isotropic_remeshing.md#features) lists them: on the command line, a
  JSON file `{"edges": [[a, b], ...], "edge_ids": [id, ...], "vertices": [[v, id], ...]}`, its
  indices vertices of the input file, each edge an edge of the input; in memory, a mesh on the
  input's vertex rows, whose edges are the curves (with an Int64 cell field `edge_id`) and whose
  points are the fixed vertices (with `vertex_id`). The ids are carried to the output: both halves
  of a split edge keep theirs. With an envelope, a tube of the same thickness holds the curves.
- [`feature_fields`](../reference/isotropic_remeshing.md#feature_fields) names per-face fields of
  a `.msh` input, labels, whose boundaries become curves; every output triangle has the value of
  the input triangle it comes from. These curves have no tube of their own unless `features` is
  given too. It reads the file, so it works on the command line and on files only.

The box again, without an envelope, its 12 sharp edges found by the angle between their two
triangles, each with an id, and its 8 corners fixed:

=== "Python"

    ```python
    --8<-- "docs/examples/python/isotropic_remeshing.py:sharp"
    ```

    In memory, the curves are a mesh:

    ```python
    --8<-- "docs/examples/python/isotropic_remeshing.py:features_memory"
    ```

    On files, a features file, and a `.vtu` output for the curves and the ids:

    ```python
    --8<-- "docs/examples/python/isotropic_remeshing.py:features_file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/isotropic_remeshing.cpp:features"
    ```

It prints `12 sharp edges -> 128 feature edges, 12 ids, 8 fixed vertices`, and every vertex is
on the box: the farthest is at distance 0. On the command line, `"features": "box_features.json"`
names the same file.

The same 12 edges are the boundaries of a label per side of the box: written in a `.msh` file as
a per-face field, `feature_fields` finds them, and the output has 128 feature edges again, each
triangle with its side's label:

```python
--8<-- "docs/examples/python/isotropic_remeshing.py:labels"
```

Both write a `.vtu`: the triangles, with the per-face fields and the point field `vertex_id`
(the fixed vertices' ids, -1 elsewhere), and `<stem>_edge.vtu` next to it, the feature edges with
their `edge_id` (-1 for a boundary of `feature_fields`). Any other format holds the triangles
only, with a warning.

### The swaps: `swap_quality`

By default an edge is swapped where that brings the valences of its four vertices closer to 6.
That leaves alone a thin triangle whose swap would not change the valences much: next to a frozen
boundary or a feature curve, where the curve itself cannot be swapped. With
[`swap_quality`](../reference/isotropic_remeshing.md#swap_quality), an edge is swapped where that
raises the worse shape quality of its two triangles. On the bunny:

```python
--8<-- "docs/examples/python/isotropic_remeshing.py:swap"
```

| | smallest angle | under 30 degrees |
|---|---|---|
| valence (default) | 0.02 | 0.76% |
| `swap_quality` | 7.41 | 0.49% |
| `swap_quality`, `boundary_eps_rel` 0.001 | 14.84 | 0.42% |

The valences are then less regular. Turn it on when the smallest angle matters more than the
valences, and with features.

### The link condition: `use_link_condition`

[`use_link_condition`](../reference/isotropic_remeshing.md#use_link_condition) (default true)
refuses a collapse that would change the topology: the genus, the boundary loops, or the
manifoldness. Turned off, a coarse target can change the surface itself: `sphere.obj` remeshed
with edges of half its diagonal ends as a single triangle instead of a tetrahedron. Turn it off
only for inputs whose topology is wrong already: on `double_sphere.obj`, two overlapping spheres
joined by non-manifold edges, the link condition refuses many collapses at a coarse target, and
leaves more vertices and thinner triangles than without it. Swaps are refused on non-manifold
edges either way.

## Outputs

- The remeshed mesh: `output` on the command line (default `out.obj`), `outputs["mesh"]` in
  memory. With features, a `.vtu` output and its `_edge.vtu`, and in memory `outputs["mesh"]`
  with the vertex field `vertex_id` and `outputs["edges"]`, the feature edges on the same vertex
  rows with `edge_id`. A result goes back in as it is: `mesh` as the input, `edges` as the
  features.
- With `report`, a JSON file with the target length, the average, smallest and largest edge
  length, the valences, the vertex and triangle counts, the non-manifold edges and vertices, the
  number of operations refused because they would fold the surface, and the run time. In memory,
  the report is returned.
- With [`history_output`](../reference/isotropic_remeshing.md#history_output), the history of the
  operations, from which [tracking](tracking.md) maps points between the input and the output.

## Recipes and pitfalls

### A label map

A mesh with a label per triangle, remeshed with the boundaries between labels kept as curves:
`topo_annots.msh`, a flat square of 2,601 vertices with six labels in the field `tag_0`. The
square's outline is in a tube rather than frozen, so that it is remeshed to the target too, and
the swaps go by quality, since the label boundaries are curves:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/isotropic_remeshing_labels.json"
    ```

    ```bash
    wmtk_app -j isotropic_remeshing_labels.json
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/isotropic_remeshing.py:label_map"
    ```

The 310 edges between labels become curves; the result has 1,177 triangles, each with its label
in `topo_annots_remeshed.vtu`, and 92 feature edges in `topo_annots_remeshed_edge.vtu`.

### Pitfalls

- **A misspelled option is ignored.** `isotropic_remeshing` runs with the default of an option it
  does not know; `wmtk_app` and the Python bindings log a warning
  (`isotropic_remeshing has no option ...`). Editors catch it beforehand with the
  [JSON Schemas](../json-schemas.md).
- **The surface drifts without an envelope.** Smoothing is only tangential to the current mesh:
  set `eps_rel` when the result has to stay close to the input, and features for edges that must
  stay sharp.
- **The target length is not reached with an envelope.** A collapse that would leave the envelope
  is refused, so where the envelope is thin compared with the target length, the mesh stays finer,
  and its triangles worse. Thicken the envelope or accept the finer mesh.
- **Thin triangles next to curves or a frozen boundary.** Turn on `swap_quality`; and
  [`cleanup_angle`](../reference/isotropic_remeshing.md#cleanup_angle), which repairs the
  triangles under a given angle at the end of every iteration.
- **The features are lost.** Only a `.vtu` output holds the feature edges, their ids and the
  per-face fields; an `.obj` holds the triangles, and the run warns.
- **`feature_fields` in memory.** It reads the fields from the input file, so the memory route
  refuses it; pass the curves as `features` instead.
- **Inconsistently oriented input.** The fold check
  ([`check_inversions`](../reference/isotropic_remeshing.md#check_inversions)) counts every flip
  of orientation as a fold, so most operations there are refused. Orient the input first.
- **The output is not next to the JSON file.** `wmtk_app` reads `input` (and `features`) relative
  to the JSON file, but writes `output` and `report` relative to the working directory.

## See also

- [Reference: every option of `isotropic_remeshing`](../reference/isotropic_remeshing.md)
- [QSlim](qslim.md), [shortest edge collapse](shortest_edge_collapse.md)
- Botsch and Kobbelt 2004, [A Remeshing Approach to Multiresolution Modeling](https://ls7-gv.cs.tu-dortmund.de/downloads/publications/2004/sgp04.pdf)
