# Topological offset

`topological_offset` adds a region at a given distance around a selection of a tagged tet mesh: a
shell around a body, inside it or both, or a band around an expression of tags (an overlap, the
curve where three regions meet) or around a surface group. It grows a band of cells from the
selection, gives them new tags, and then optimizes the mesh until the band's outer boundary, the
front, lies at the target distance. The band's topology is fixed when it is built and kept
throughout ([Topological Offsets, ACM TOG 2025](https://dl.acm.org/doi/10.1145/3731157)).

Use it to make a coating, a gap, or a contact layer of a given thickness around regions that
already exist in a tagged mesh, such as one from [simwild](simwild.md). To separate or join
regions that touch only along an edge or at a vertex, use
[manifold extraction](manifold_extraction.md), which carves the same kind of band without the
optimization.

!!! note "What runs in this version"
    The component runs on tet meshes only: a triangle mesh stops with `2D input refused: the 2D
    offset is not up to date`. The optimization is under active development; many of its options
    are marked `EXPERIMENTAL_` in the reference and change often. This page covers the stable
    options.

## A first run

`127891.msh` is a bar (`127891`, 342 tets) in a box of background, 1,556 tets in all. A band of
distance 0.4 around it, outside, built but not optimized (`optimize_offset` false), which takes a
fraction of a second:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/topological_offset.json"
    ```

    ```bash
    wmtk_app -j topological_offset.json
    ```

    The input must be a `.msh`. The result is `127891_offset.msh`, with the band tagged `offset`.

=== "Python"

    ```python
    --8<-- "docs/examples/python/topological_offset.py:memory"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/topological_offset.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/topological_offset.cpp:run"
    ```

    Link `wmtk::topological_offset`.

It prints `1556 -> 4802 tets, 2094 in the band`. The log says how the band was placed and that it
passed its checks:

```text
[construction] construction_mode max_marchable_fallback: maximum marchable distance 0.49254 over 592 marched edges; target_distance 0.4 is below it, so the march traces to the target
Offset region manifold check passed.
connected components check passed. (# components=1)
```

The front is a closed surface of 1,180 triangles. Its 592 vertices lie at the target: their
distance to the bar is between 0.990 and 1.000 times 0.4. Its triangles do not: they are chords
between those vertices, and their centroids are between 0.54 and 1.02 times the target (0.93 on
average). The band's cells are thin (max AMIPS 179, against 19.2 in the input). Moving the whole
front onto the target and restoring the quality is the job of the
[optimization](#the-optimization-optimize_offset-front_conv_rel).

## How the band is built

1. **The complex.** `offset_selection` picks the input complex: cells, and the faces, edges and
   vertices between them.
2. **The embedding.** Cells are split so that no edge, face or tet outside the complex has all
   its vertices on it (the log says `Input complex/offset simplicially embedded: TRUE`).
3. **The march.** Every edge with one end on the complex is split at the target distance from
   the complex, found by sphere tracing. That needs the edge's far end to be farther than the
   target. The smallest such distance over all the edges is the maximum marchable distance; when
   the target is not below it, every edge is split at half of it instead, and the front starts
   short of the target.
4. **The band.** The cells around the new vertices are the band, one cell thick, and get the
   output tags.
5. **The optimization** (unless `optimize_offset` is false): turns of splits, collapses, swaps
   and smoothing that move the front onto the target distance and refine where it is not, with
   TetWild passes for quality before and after them.

The band's topology is the one the march gives it, and every later operation keeps it.

## Key settings

### Which side: `offset_selection`, `offset_in`, `offset_out`

[`offset_selection`](../reference/topological_offset.md#offset_selection) names what to offset. A
single tag name, as in the first run, is single body mode: the body is that region, and
[`offset_out`](../reference/topological_offset.md#offset_out) (default true) and
[`offset_in`](../reference/topological_offset.md#offset_in) (default false) say on which side of
its boundary the band grows. At least one must be true.

```python
--8<-- "docs/examples/python/topological_offset.py:sides"
```

| side | band tets | the march traces to |
|---|---|---|
| `offset_out` | 2,094 | 0.4, the target |
| `offset_in` | 2,876 | 0.0973, half of the inside's maximum marchable distance, 0.195 |
| both | 4,970 | 0.0973 |

Inside the bar the maximum marchable distance is 0.195, below the target, so the inward band
starts at a quarter of the target and the optimization has to carry it the rest of the way. The
default `offset_selection`, `!_`, left the example mesh unchanged: set it.

### Expressions

Any other [`offset_selection`](../reference/topological_offset.md#offset_selection) is an
expression over tag names, with `&`, `|`, `!` and parentheses, as in simwild (`ambient` is the
background). The complex is every tet whose tags satisfy it, and every face, edge and vertex
whose surrounding tets' tags, together, satisfy it. The band grows on every side of the complex;
`offset_in` and `offset_out` are not read. On `double_sphere.msh`, two overlapping spheres
`tag_0` and `tag_1`, with a target of 0.05:

```python
--8<-- "docs/examples/python/topological_offset.py:expressions"
```

| `offset_selection` | the complex | band tets |
|---|---|---|
| `tag_0 & tag_1` | the overlap, a lens of tets | 2,716 |
| `ambient & tag_0 & tag_1` | the circle where the two surfaces cross: 28 edges | 886 |

A complex of edges alone, like the circle, is built but not optimized: with the optimization the
run stops with `the energy's band term needs an input complex of triangles only`. Set
`optimize_offset` false for it.

### Surfaces: `EnvelopeSurface` and face tags

A `.msh` with an `EnvelopeSurface` triangle group (simwild writes one) has that group as a tag of
its own name, and `offset_selection` can name it alone: the complex is the faces of the mesh that
lie within [`envelope_size_rel`](../reference/topological_offset.md#envelope_size_rel) of the
group, and the band grows on both sides of them. This is how to offset an open sheet. The log
says how many faces carry it:

```text
Surface group 'EnvelopeSurface' (tag 3): 330 triangles; 310 of the mesh's faces lie inside its envelope (eps 0.0836779) and carry the tag
```

In `127891.msh` the group is the bar's own surface, so `"offset_selection": "EnvelopeSurface"`
gives the same 4,970 band tets as `offset_in` and `offset_out` together. A face tag (simwild's
`input_type` `surface`) is selected by its name in the same way, with `optimize_offset` false
only.

### Tags of the band: `offset_output_tags`, `protected_tags`

[`offset_output_tags`](../reference/topological_offset.md#offset_output_tags) (default
`["newtag"]`) are the tags every band cell gets, replacing the tags it had; a name the mesh does
not have is added as a new region. `[]` makes the band background, which carves a gap.
[`protected_tags`](../reference/topological_offset.md#protected_tags) are kept on the band cells
that have them, alongside the output tags. `ambient` can be neither (it is ignored, with a
warning). On the two spheres, the band around `tag_0` runs through `tag_1`:

```python
--8<-- "docs/examples/python/topological_offset.py:protected"
```

The band is 3,998 tets either way; `tag_1` keeps 3,539 tets without protection and 4,729 with it:
1,190 band cells stay in `tag_1` as well.

### The distance: `target_distance_rel`, `target_distance`

[`target_distance_rel`](../reference/topological_offset.md#target_distance_rel) (default 0.01) is
the target relative to the diagonal of the whole mesh's bounding box, background included;
[`target_distance`](../reference/topological_offset.md#target_distance), when positive, replaces it
with an absolute length. The box of `127891.msh` has a diagonal of 83.7, so the default target is
0.837, above the maximum marchable distance, 0.493: the march puts the front at 0.246, 0.29 of the
target, and the optimization has to grow it. On this mesh, with the other options at their
defaults (and four threads), that optimization had not converged after 22 turns, when the run was
stopped.

A target below the maximum marchable distance is traced exactly, as in the first run. That
distance is the one from the complex to the nearest far end of an edge that leaves it, so it is
set by the cells around the selection, and the construction does not change them beforehand: the
input's resolution decides what the construction can reach.

### The field: `offset_field`

[`offset_field`](../reference/topological_offset.md#offset_field) is the field the front is placed
on. `euclidean`, the default, is the distance to the complex, and the only one available: `smooth`
stops with `offset_field "smooth" is not available in 3D`.

### The optimization: `optimize_offset`, `front_conv_rel`

[`optimize_offset`](../reference/topological_offset.md#optimize_offset) (default true) runs the
optimization after the construction: up to
[`max_rounds`](../reference/topological_offset.md#max_rounds) (default 40) turns of splits,
collapses, swaps and smoothing that place the front, a TetWild pass before them on the
constructed mesh, and a final one if the max AMIPS is over
[`stop_energy`](../reference/topological_offset.md#stop_energy) (100). After each turn, each
front vertex is measured on the front triangles around it: the root mean square of
(distance − target) / target over sample points on them. The loop stops when every vertex is
within the bar, [`front_conv_rel`](../reference/topological_offset.md#front_conv_rel) times the
target (default 0.025); around a vertex over the bar, the target edge length is halved. Each
turn logs the measure in units of the bar:

```text
======== turn 5 / 5: max AMIPS 117.9 (stop 100) | ring measure max 3.766x the bar (avg 0.3237x) ...
```

The optimization is most of the cost. The first run's offset, optimized for five turns with a bar
of 20% of the target and four threads:

```python
--8<-- "docs/examples/python/topological_offset.py:optimize"
```

It prints `converged: False`, 21,683 to 24,423 tets in four runs (the threads change the order of
the operations), and a front distance error of 0.016 to 0.024 on average and 0.4 at most.
Measured back to back, these five turns take about 400 times as long as the construction, and the
time per turn grows with the mesh. On one result (23,148 tets), 84% of the front's vertices are
within 2.5% of the target and 83% of its triangles' centroids within 5% (55% after the
construction); the max AMIPS is 117.9. A few front vertices are still off by the whole target, and the log warns
that some 170 to 200 of the front's 2,300 to 2,500 triangles are folded (`[foldover]`). Ten turns
with the same bar end at 100,131 tets, still not converged (worst vertex 4.9 times the bar). None
of the optimized runs made for this page converged.

With `optimize_offset` false, the run stops after the march: the front's vertices are at the
target (or at half the maximum marchable distance), the report has no convergence fields, and the
mesh's diagnostic fields (`input_distance` and others) are NaN.

### The checks: `check_manifoldness`

With [`check_manifoldness`](../reference/topological_offset.md#check_manifoldness) (default true),
the region made of the complex and the band must be manifold after the construction, or the run
stops with `OFFSET REGION IS NOT MANIFOLD`; after the optimization it is checked again, and a
failure is logged as an error. Whatever this option says, the mesh's connected components are
counted before and after the construction, and a change stops the run
(`connected components check passed` in the log).

## Outputs

- The mesh: `output` on the command line (default `out`, written as `out.msh`), `outputs["mesh"]`
  in memory. Its tags are the input's, with the output tags on the band (and an empty
  `EnvelopeSurface` column when the input has that group). The vertex field `labels` is 1 on the
  complex, 2 on the vertices the march added, 0 elsewhere: in the first run, 157, 592 and 195. The
  other fields are diagnostics of the optimization.
- In memory, `outputs["offset"]`: the front, as triangles on the mesh's vertices, after the
  optimization (2,462 triangles in one five-turn run); a construction-only run returns it
  without triangles. `outputs["envelope"]` is the input's `EnvelopeSurface` group, passed on.
- With `report`, a JSON file with the vertex, edge, face and tet counts before and after, the
  threads and the run time; after the optimization also `converged` and `optimization_metrics`,
  whose `avg_l2_dist_err` and `max_l2_dist_err` are the front's distance error. In memory, the
  report is the second return value.
- With [`save_vtu`](../reference/topological_offset.md#save_vtu), `.vtu` files for ParaView.

## Recipes and pitfalls

- **A misspelled or outdated option is an error.** Unlike most components, `topological_offset`
  refuses an option its spec does not have: `No rule matched for "/front_conv_criterion"`
  (`wmtk_app` logs `topological_offset has no option ...` first). Configurations written for
  earlier versions, with `front_conv_criterion`, `offset_gradient_rel` or `offset_residual_rel`,
  fail this way. Start from the [reference](../reference/topological_offset.md).
- **The run does not converge.** The result is written anyway, with a warning `Optimization did
  not converge` and `converged` false in the report; check it before using the offset.
  [`throw_on_nonconvergence`](../reference/topological_offset.md#throw_on_nonconvergence) makes it
  an error instead.
- **The target is beyond the cells.** When the log says the march traces to half of the maximum
  marchable distance, the constructed front is short of the target, and only the optimization
  carries it out, which is slow. Pick a target below that distance where the offset allows it;
  [`growth_mode`](../reference/topological_offset.md#growth_mode) sets how the optimization grows
  the front.
- **A surface group that does not lie on mesh faces.** The group's faces are found
  geometrically. On `double_sphere.msh`, only 129 of the mesh's faces lie within the group's
  envelope, in 36 pieces, so the complex is a scatter of patches; the log line above says so.
- **Parallel runs differ.** With [`num_threads`](../reference/topological_offset.md#num_threads)
  above 0 the optimization's result changes slightly from run to run (the tet counts above);
  0 runs sequentially.
- **For prismatic meshes**, [`save_offset_correspondence`](../reference/topological_offset.md#save_offset_correspondence)
  records which input vertex each marched vertex came from; it needs `optimize_offset` false.

## See also

- [Reference: every option of `topological_offset`](../reference/topological_offset.md)
- [Manifold extraction](manifold_extraction.md): the same construction around non-manifold edges
  and vertices
- [SimWild](simwild.md): tagged meshes, and the tag expressions
- [Topological Offsets, ACM TOG 2025](https://dl.acm.org/doi/10.1145/3731157)
