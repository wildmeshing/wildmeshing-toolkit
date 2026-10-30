# TetWild

`tetwild` fills the inside of triangle surfaces with tetrahedra, taking the surfaces as they come:
open, self-intersecting, non-manifold, inconsistently oriented. It simplifies the input inside an
envelope, inserts it exactly into a Delaunay tetrahedralization of its bounding box, so that every
input triangle becomes a union of tet faces, and then improves the tets' shape while the inserted
surface stays within a distance \(\varepsilon\) of the input
([Hu et al. 2018](https://cims.nyu.edu/gcl/papers/2018-TetWild.pdf),
[2020](https://arxiv.org/abs/1908.03581)). A filter can remove the tets outside at the end.

Use it to get a tet mesh for simulation from a surface that is not clean enough for a
constrained Delaunay mesher, or several surfaces at once. The surface of the result
approximates the input within \(\varepsilon\); it does not reproduce its triangles. For curves in
the plane, use [TriWild](triwild.md); to keep curves and points inside the volume, see
[TetWild with features](workflows/tetwild-features.md).

## A first run

A snowman from Thingi10K (`thingi_366725.stl`, 1,617 triangles), as found in the wild: a body with
holes, the eyes and buttons as five separate open patches over them, eleven edges where
neighbouring triangles disagree on their orientation, and a non-manifold edge.

=== "Command line"

    ```json
    --8<-- "docs/examples/json/tetwild.json"
    ```

    ```bash
    wmtk_app -j tetwild.json
    ```

    The input path is relative to the JSON file; the outputs, the tet mesh `snowman_final.msh`
    among them (see [outputs](#outputs)), are written to the working directory.

=== "Python"

    ```python
    --8<-- "docs/examples/python/tetwild.py:memory"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/tetwild.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/tetwild.cpp:run"
    ```

    Link `wmtk::tetwild`.

The 971 input vertices (after welding the STL's corners) are simplified to about 670. The
surface is inserted into a Delaunay mesh of its bounding box (about 19,000 tets), which the
insertion cuts into about 58,000, and the optimization runs until every tet's energy is below 10.
Of the result, the filter keeps the inside: over twelve runs, 8,000 to 9,500 tets on 1,900 to
2,230 vertices, the largest energy between 9.5 and 10, the average 3.65 to 3.69 (see
[threads](#threads-and-reproducibility)). A run takes seconds.

<div>
<wmtk-viewer example="tetwild"></wmtk-viewer>
</div>

The input on the left, the result on the right: drag to rotate, scroll to zoom; on the tet mesh, move the cut with the slider. Every example is in the [gallery](../gallery/index.md).

## Key settings

The numbers below come from the `sweep` section of the example, which changes one option at a
time against the first run, and from runs with the same options and other values:

```python
--8<-- "docs/examples/python/tetwild.py:sweep"
```

They run with 4 threads, so counts vary by several percent from run to run; a range is over
several runs, a single number from one.

### The inputs: `input`, `input_names`, `output`

[`input`](../reference/tetwild.md#input) is a list of triangle meshes (OBJ, STL, MSH, ...), up to
64. They are read, welded (vertices that coincide exactly are merged, see
[`remove_duplicate_eps`](../reference/tetwild.md#remove_duplicate_eps)) and meshed together: their
intersections are resolved exactly. With more than one input, each tracked face is held to the
envelope of its own input. Each input gets a winding-number field on the tets: `wn_0`, `wn_1`,
..., or, with [`input_names`](../reference/tetwild.md#input_names), `wn_<name>` in the VTU file
and the memory outputs. [`output`](../reference/tetwild.md#output)
is the file name without extension (default `out`).

In memory, `input` names entries of the meshes dictionary.

### The envelope: `eps_rel`

[`eps_rel`](../reference/tetwild.md#eps_rel) (default 0.001) is the distance the output's surface
may be from the input, relative to the bounding box diagonal. The input is first simplified within
half of it, so detail smaller than \(\varepsilon\) may disappear; the optimization then keeps
every tracked face within \(\varepsilon\) of the input. A thinner envelope keeps the surface
closer and leaves the optimizer less room, so it takes more tets to reach the same quality:

| `eps_rel` | \(\varepsilon\) | tets | largest distance to the input |
|---|---|---|---|
| 0.0005 | 0.059 | 14,183 | 0.033 |
| 0.001 (default) | 0.117 | 8,000–9,500 | 0.065–0.067 |
| 0.002 | 0.235 | 6,817 | 0.140 |
| 0.005 | 0.586 | 6,400–6,600 | 0.33–0.36 |

The distance is what [`DEBUG_hausdorff`](../reference/tetwild.md#DEBUG_hausdorff) measures and the
report stores as `hausdorff`: the largest distance from a point of the output's surface to the
input, sampled. It stays below \(\varepsilon\) in every run. The other direction, `coverage`, is
not bounded: the simplification may remove a feature thinner than \(\varepsilon\).

### The edge length: `length_rel`

[`length_rel`](../reference/tetwild.md#length_rel) (default 0.05) is the target edge length,
relative to the bounding box diagonal. Edges longer than 4/3 of it are split, and edges shorter
than 4/5 of it are collapsed when the envelope and the quality allow. It sets the size of the tets
away from the surface; near the surface, the surface's own detail and the envelope do:

| `length_rel` | tets |
|---|---|
| 0.1 | 4,300–6,500 |
| 0.05 (default) | 8,000–9,500 |
| 0.025 | 47,315 |

Halving the length multiplies the count by about five; doubling it removes only a third to a half
of the tets, because most of the snowman's tets are near its surface. A finer mesh takes proportionally longer:
the run at 0.025 took about five times as long as one at the default.

### The quality target: `stop_energy`

The shape of a tet is measured by its AMIPS energy: 3 for a regular tetrahedron, growing without
bound as the tet flattens (see [Quality](../concepts/quality.md)). The optimization stops when the
largest energy is below [`stop_energy`](../reference/tetwild.md#stop_energy), or after
[`max_iterations`](../reference/tetwild.md#max_iterations) (80):

| `stop_energy` | tets | max energy | average | iterations |
|---|---|---|---|---|
| 100 (default) | 6,100–6,500 | 22.3–58.2 | 3.83–4.02 | 1–3 |
| 20 | 6,518 | 18.1 | 3.72 | 6 |
| 10 | 8,000–9,500 | 9.5–10 | 3.65–3.69 | 7–10 |
| 5 | 53,353 | 4.91 | 3.50 | 32 |

The average hardly moves; the target decides the worst tets, which are what a simulation suffers
from. At the default the result is whatever the first iterations leave below 100: the largest
energy ranged from 22 to 89 over nine runs. The examples ask for 10. Below that the cost climbs
quickly: the run at 5 took six to eight times as long as runs at 10, for six times the tets. A
run that ends at `max_iterations` without reaching the target still writes its mesh;
[`throw_on_fail`](../reference/tetwild.md#throw_on_fail) makes it an error instead.

### The outside: `filter`

The tet mesh covers the bounding box of the input, grown by a margin.
[`filter`](../reference/tetwild.md#filter) decides what is kept (see
[Embedding](../concepts/embedding.md) for how each works). The default, `none`, keeps everything.
The three filters agree on a closed surface and differ on the others (these runs are coarse,
`length_rel` 0.1, and sequential, so they repeat exactly):

```python
--8<-- "docs/examples/python/tetwild_filters.py:open"
```

| Surface | `none` | `flood` | `input` | `tracked` |
|---|---|---|---|---|
| sphere, closed | 16,383 tets, volume 5.118 | 4,434 tets, volume 1.404 | 4,434, 1.404 | 4,434, 1.404 |
| sphere without its top cap (80 of 760 triangles) | 15,111, 4.637 | error: `Empty Output after Filter!` | 3,864, 1.330 | 3,864, 1.330 |

The closed sphere's volume is 1.404, and all three filters keep exactly the tets inside it. On the
open sphere, `flood` finds a single region, the box, and removes it all; the winding numbers still
tell inside from outside away from the hole, and both keep the tets whose winding number is above
1/2. `tracked` is documented to need closed components; it worked on this hole, which is not a
guarantee.

They also differ in what they read. `flood` uses only the surface, not its orientation; `input`
and `tracked` use the orientation. On `double_sphere.obj`, two intersecting spheres whose triangles
are oriented at random:

```python
--8<-- "docs/examples/python/tetwild_filters.py:orientation"
```

`flood` keeps 4,192 tets (volume 1.824, the union of the two balls), `tracked` 3,956 (1.720), and
`input` 143 (0.040). `tracked` repairs triangles flipped against their neighbours, which recovers
most of the volume here; `input` evaluates the input as it is. And where both work, `input`
judges each tet by the input surface rather than by the surface the mesh conforms to, so the
boundary of what it keeps can stray from the tracked surface: on the Octocat (default options),
31 faces of the kept tets' boundary are not tracked faces with `input`, none with `tracked` or
`flood`. `flood`, for its part, removes only the region outside, so it keeps cavities that a
winding number would call outside.

In short: `tracked` for consistently oriented inputs, closed or nearly so; `flood` for closed
inputs whose orientation cannot be trusted; `input` for open inputs, as in the
[tracking tutorial](tracking.md). The winding numbers are written to the output in any case (see
[outputs](#outputs)), so a `none` run can be filtered afterwards.

### Feature curves and points: `input_edges`, `input_points`

[`input_edges`](../reference/tetwild.md#input_edges) lists edge meshes whose edges are inserted as
chains of tet edges, and [`input_points`](../reference/tetwild.md#input_points) files whose
vertices are inserted as tet vertices; the optimization keeps both in place. The
[TetWild with features](workflows/tetwild-features.md) workflow shows them on a cube, and how to
check what was kept.

### Topology: `preserve_topology`

By default the simplification and the optimization may change the surface's topology: close holes
and slits narrower than the envelope, and make the surface non-manifold.
[`preserve_topology`](../reference/tetwild.md#preserve_topology) forbids it: only exactly
coincident vertices are merged, the simplification and the operations check the link condition,
and the run compares the Euler characteristics of the input surface and the output's surface at
the end, logging a warning when they differ (an error with `throw_on_fail`). With a filter, the
output's surface is the boundary of the kept tets, which is closed, so the check only makes sense
for closed inputs.

Use it on clean inputs whose topology matters. On the Octocat (one closed surface, Euler
characteristic 2), with `filter` `tracked` and `stop_energy` 10, it takes 13 iterations and
10,095 tets where the same run without it takes 7 and 8,122; the Euler characteristic is 2 in
both. On the snowman it does not converge: after 15 iterations the largest energy is still 4,135,
with 75,001 tets.

### Coarsening: `coarsen_pass`

The collapses of the optimization judge each candidate on the mesh right after it, before smoothing
has relaxed it, so a converged mesh keeps vertices it does not need.
[`coarsen_pass`](../reference/tetwild.md#coarsen_pass) runs a last pass that takes such collapses
and keeps each one if the worst tet around it is no worse than before. The largest energy cannot
rise; the average does:

| | tets | max energy | average |
|---|---|---|---|
| first run | 8,000–9,500 | 9.5–10 | 3.65–3.69 |
| with `coarsen_pass` | 2,330–2,530 | 9.97 | 4.70–4.92 |

About a quarter of the tets for the same worst case, at about a tenth more time. The report's
`coarsen_max_energy_before` and `coarsen_max_energy_after` were equal in every run that recorded
them (9.97); `coarsen_t_before` and `coarsen_t_after` count the whole box's tets, before the
filter (37,932 and 5,483 in one run).

### Correspondences: `history_output`

[`history_output`](../reference/tetwild.md#history_output) records every change to the tracked
surface, from the input files to the output, and [tracking](tracking.md) maps points between the
input surface and the tet mesh's surface from it. With
[`volume_history_output`](../reference/tetwild.md#volume_history_output) the tets around each
change are recorded too, for a map of the whole volume. The tracking tutorial chains isotropic
remeshing and tetwild this way, and lists the options that keep the map a bijection.

### Threads and reproducibility

[`num_threads`](../reference/tetwild.md#num_threads) 0 runs sequentially and gives the same mesh
every time (8,358 tets for the first run's options). With threads, the order of the operations
depends on the schedule, and the result changes from run to run: 8,000 to 9,500 tets. Run back to
back, the sequential run took about 2.3 times as long as one with 4 threads.

## Outputs

On files, `output` names:

- `snowman_final.msh`: the tet mesh. Each tet has its energy (`energy`), `winding_number_input`
  (the winding number of all inputs together at its centroid), one `wn_<i>` per input, and, when
  they were computed, `winding_number_tracked` and the flood-fill region `part`. The vertex field
  `sizing` holds the sizing scalar of the optimization.
- `snowman.vtu`: the same tets for ParaView, with `t_energy`, the winding numbers (`wn_<name>`
  with `input_names`) and the vertex fields `sizing_field`, `is_rounded`, `on_surface`,
  `on_open_boundary` and `order`.
- `snowman_surf.vtu`: the tracked surface, the tet faces that lie on the input.
- `snowman_surface.obj`: with a filter, the boundary of the tets kept; with `filter` `none`, the
  tracked surface.
- `snowman_simplified_input.obj`: the input after the simplification, as inserted.
- `snowman_features.obj`: with feature curves or points, the tracked curves (`l`) and the
  anchored points (`p`).
- With [`report`](../reference/tetwild.md#report), a JSON file with the vertex and tet counts
  (`#v`, `#t`), `max_energy`, `avg_energy`, `#iterations`, `eps`, the coarsening statistics, and
  `hausdorff` and `coverage` with `DEBUG_hausdorff`, `input_euler_characteristic` and
  `output_euler_characteristic` with `DEBUG_euler` or `preserve_topology` (-1 or empty when not
  measured), and `time`, the seconds from the insertion to the filter.

The winding-number fields let a `filter` `none` run be split afterwards: a tet is inside an input
where its `wn_<i>` is above 1/2.
[`compute_tracked_winding_number`](../reference/tetwild.md#compute_tracked_winding_number) and [`compute_flood_fill`](../reference/tetwild.md#compute_flood_fill) add the other two fields
without filtering; [`skip_winding_number`](../reference/tetwild.md#skip_winding_number) leaves the
input's out when no filter needs them.

In memory, the outputs are `mesh` (the tets, with the fields of the VTU file), `surface` (the
tracked triangles, on the same vertices as `mesh`), `envelope` (the welded input the envelope is
built around), and `features` with feature curves or points. The report holds the same keys as the
file, and the history's bytes under `history` when `history_output` is set.

## Pitfalls

- **The default filter keeps the box.** `filter` `none` returns the whole bounding box, inside
  and outside; set a filter to get the inside.
- **`flood` on an open input** removes everything and fails with `Empty Output after Filter!`.
- **Everything is relative to the bounding box.** `eps_rel` and `length_rel` multiply the
  diagonal of the box around all the inputs, feature curves and points included. A stray triangle
  or point far from the rest grows the box, and with it \(\varepsilon\) and the edge length.
  Remove it, or compensate with smaller values.
- **`preserve_topology` on a dirty input** keeps every tiny handle, gap and sliver of the input,
  and the optimization may not converge around them. Repair the input first (see
  [repair before meshing](workflows/repair-before-meshing.md)).
- **A misspelled option is ignored.** `wmtk_app` and the Python bindings log a warning
  (`tetwild has no option ...`) and run with the defaults. Editors catch it beforehand with the
  [JSON Schemas](../json-schemas.md).

## See also

- [Reference: every option of `tetwild`](../reference/tetwild.md)
- [Embedding](../concepts/embedding.md): the simplification, the insertion, the tracked surface
  and the filters; [Quality](../concepts/quality.md): the energy, the optimization loop and the
  coarsening pass; [Robust geometry](../concepts/robust-geometry.md): the envelope.
- [TetWild with features](workflows/tetwild-features.md), [TriWild](triwild.md),
  [tracking](tracking.md).
- Hu et al. 2018, [Tetrahedral Meshing in the Wild](https://cims.nyu.edu/gcl/papers/2018-TetWild.pdf);
  Hu et al. 2020, [Fast Tetrahedral Meshing in the Wild](https://arxiv.org/abs/1908.03581)
