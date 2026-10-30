# SimWild

`simwild` builds and edits meshes with several regions. Given several closed surfaces (3D) or
closed curves (2D), it meshes them all into one tet or triangle mesh of their bounding box, with a
region per input: a cell belongs to every input it lies inside, so overlaps are regions too, and
the cells inside no input are the background. It then optimizes the mesh as
[TetWild](tetwild.md) and [TriWild](triwild.md) do, with every interface between regions held
within \(\varepsilon\) of the inputs, and with an edge length and a quality target that can differ
per region. Given a mesh that already has regions, its `operation` either remeshes it or edits its
regions: fills holes, closes gaps, removes small pieces, splits or renames overlaps.

Use it when the regions are the point: several materials, organs or parts in one conforming mesh,
for simulation. For one solid, with the outside removed, [TetWild](tetwild.md) and
[TriWild](triwild.md) are the direct route; to add a region at a given distance around another, use
[topological offset](topological_offset.md) on simwild's output (see the
[multi-region workflow](workflows/regions.md)).

## A first run

Two copies of one sphere (radius 0.7), the second moved 0.3 along \(x\) by a transformation matrix,
named `a` and `b`:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild.json"
    ```

    ```bash
    wmtk_app -j simwild.json
    ```

    Paths in the file are relative to the file. The tet mesh is `two_spheres.msh`.

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild.py:memory"
    ```

    `regions` counts the cells of each combination of regions:

    ```python
    --8<-- "docs/examples/python/simwild.py:regions"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/simwild.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/simwild.cpp:run"
    ```

    `regions` counts the cells of each combination of regions:

    ```cpp
    --8<-- "docs/examples/cpp/simwild.cpp:regions"
    ```

    Link `wmtk::simwild`.

The two surfaces are inserted exactly into a tet mesh of their bounding box, grown by 1/15 of its
diagonal on every side; every tet is tagged with the spheres whose winding number at its centroid
says it is inside. The mesh is then simplified a little and optimized until every tet's energy is
below 10, which it reaches in its first iteration. Over a dozen runs it has 19,900 to 21,800 tets,
the largest energy between 6.0 and 8.6, and a run takes a few seconds. The regions, in one of them:

```text
{'a': 1582, 'a & b': 2954, 'ambient': 14766, 'b': 1382}
```

`a` alone, `b` alone, the lens where they overlap, and the background, which the output labels
`ambient`. The counts move by several percent from run to run: the examples use 4 threads, and
the order of the operations depends on the schedule; `num_threads` 0 runs sequentially and repeats
exactly.

### Curves in 2D

The same in the plane, from five closed curves, the inputs of the [TriWild tutorial](triwild.md):
the ellipse lies inside the rectangle, the others overlap in pairs.

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_curves.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild.py:curves"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/simwild.cpp:curves"
    ```

An OBJ with `l` records and no `f` record is a curve network, so simwild takes the 2D route on its
own ([`dimension`](../reference/simwild.md#dimension) forces it). Over ten runs the result has
1,840 to 2,220 triangles, the largest energy below 8, in a fraction of a second; in one of them:

```text
{'ambient': 1196, 'blob1': 270, 'blob1 & blob2': 14, 'blob1 & rectangle': 9, 'blob2': 192,
 'blob2 & circle': 33, 'circle': 50, 'circle & rectangle': 14, 'ellipse & rectangle': 9,
 'rectangle': 121}
```

The ellipse has no cell of its own: every cell inside it is inside the rectangle too.

## Regions and tag expressions

A region is a named set of cells; a cell belongs to any number of them. The output carries them as
the `tags` attribute of the tets (triangles in 2D): one column per region, labelled with its name,
and a last column, `ambient`, for the cells in none. simwild reads `ambient` back as no region at
all. [Regions](../concepts/regions.md) explains the format and the expression language in general;
in simwild, options select cells with these expressions:

| Expression | Selects the cells |
|---|---|
| `a` | in region `a`, whatever other regions they are in |
| `a & b` | in both `a` and `b` |
| `a | b` | in `a`, in `b`, or in both |
| `!a` | not in `a` |
| `_` | in no region: the background |
| `!_` | in at least one region |

`!` binds tightest, then `&`, then `|`; parentheses group. The background is `_`, not `ambient`:
`ambient` is a label in the file, and simwild warns that the region `ambient` does not exist and
selects nothing.

Not every option takes every expression:

- [`sizing_field`](../reference/simwild.md#sizing_field) and
  [`quality_field`](../reference/simwild.md#quality_field) take any expression.
- The region edits take intersections only, `a` or `a & b`
  ([`fill_holes_tags`](../reference/simwild.md#fill_holes_tags),
  [`tight_seal_tag_sets`](../reference/simwild.md#tight_seal_tag_sets),
  [`keep_lcc_tags`](../reference/simwild.md#keep_lcc_tags),
  [`resolve_overlaps_tags`](../reference/simwild.md#resolve_overlaps_tags),
  [`replace_tags_in`](../reference/simwild.md#replace_tags_in) and
  [`replace_tags_out`](../reference/simwild.md#replace_tags_out)); `|` or `!` there is an error.
- [`tag_priority`](../reference/simwild.md#tag_priority) takes region names.

A name the mesh does not have is reported with a warning. Where an expression selects cells, it
selects nothing. In [`replace_tags_out`](../reference/simwild.md#replace_tags_out) and
[`tag_priority`](../reference/simwild.md#tag_priority) it creates the region instead
(`Tag name ... does not exist! Adding new tag.`): that is how `replace_tags` makes a new region,
and how a misspelled name in `tag_priority` goes unnoticed.

## Key settings

The numbers below come from the sections of the example that change one option at a time against
the first run, and from runs with the same options; a range is over two or more runs with 4
threads:

```python
--8<-- "docs/examples/python/simwild.py:sweep"
```

### The inputs: `input`, `input_names`, `input_type`

[`input`](../reference/simwild.md#input) is a list of files: closed triangle surfaces in 3D,
closed curve networks in 2D, one region each. Each is named by the entry at the same position in
[`input_names`](../reference/simwild.md#input_names), `tag_0`, `tag_1`, ... without one. A tagged
`.msh` as the first input is a mesh to edit or remesh instead (see
[Editing regions](#editing-regions-operation)). The same file may appear twice, as the sphere does
here, with [`input_transform`](../reference/simwild.md#input_transform) placing each copy: a 4 by 4
homogeneous matrix per input in 3D, 3 by 3 in 2D, `[]` for the identity.

An input is inside or outside by its winding number at each cell's centroid, so it should be
closed. [`input_type`](../reference/simwild.md#input_type) `surface` takes an input as a surface
instead (3D only): it tags the faces it covers rather than the tets it encloses, and it may be
open. [`split_connected_components`](../reference/simwild.md#split_connected_components) makes one
region per connected component of a file.

### The envelope: `eps_rel`

The interfaces between regions stay within \(\varepsilon\) of the inputs, where \(\varepsilon\) is
[`eps_rel`](../reference/simwild.md#eps_rel) times the diagonal of the mesh's bounding box. That box
includes the padding around the inputs: the spheres span \([-0.7, 1.0] \times [-0.7, 0.7]^2\), the
mesh \([-0.874, 1.174] \times [-0.874, 0.874]^2\), a diagonal of 3.21. The default, 0.002, is
twice TetWild's.

| `eps_rel` | \(\varepsilon\) | tets | interface triangles | farthest interface point from the input |
|---|---|---|---|---|
| 0.01 | 0.032 | 19,900 to 21,800 | 1,340 to 1,430 | 0.017 |
| 0.002 (default) | 0.0064 | 24,000 to 25,500 | 2,160 to 2,290 | 0.0034 |

The farthest point is over the interfaces' vertices and triangle centroids, against the input
triangles, in one run each. The thinner envelope costs elements on the interfaces, and the largest
energy ends close to the target (9.6 to 9.99 for 10).

### The edge length: `length_rel`

[`length_rel`](../reference/simwild.md#length_rel) (default 0.05) is the target edge length,
relative to the same diagonal; [`length`](../reference/simwild.md#length) sets it in model units.
Halving it from the default multiplies the number of tets by eight:

| `length_rel` | tets | interface triangles |
|---|---|---|
| 0.1 | 8,400 to 8,900 | 760 to 790 |
| 0.05 (default) | 19,900 to 21,800 | 1,340 to 1,430 |
| 0.025 | 168,000 to 177,000 | 4,950 to 5,170 |

Measured back to back, the run at 0.025 took about three times as long as the one at 0.05.

### Edge length per region: `sizing_field`

[`sizing_field`](../reference/simwild.md#sizing_field) is a list of `{"tags": expression,
"length_rel": ...}` (or `"length"`) entries. A cell takes the length of the last entry whose
expression it matches, the global length without one, and a vertex the smallest length of its
cells. Most of the first run's tets are background; a coarser background, then a finer overlap:

```python
--8<-- "docs/examples/python/simwild.py:sizing"
```

| `sizing_field` | tets | `ambient` | `a & b` |
|---|---|---|---|
| none | 19,900 to 21,800 | 14,000 to 14,800 | 2,740 to 3,050 |
| `_`: 0.1 | 12,300 to 13,300 | 6,600 to 7,100 | 2,810 to 3,150 |
| `a & b`: 0.025 | 43,400 to 47,500 | 14,900 to 16,100 | 21,200 to 23,500 |

The cells next to the overlap are refined too, since a vertex takes the smallest length of its
cells: `a` and `b` alone grow from about 1,500 tets each to 3,600 to 4,000.

### The quality target: `stop_energy`, `quality_field`

The optimization stops when every cell's energy (AMIPS, 3 for the regular tet, 2 for the
equilateral triangle, see [Mesh quality](../concepts/quality.md)) is below its target, or after
[`max_iterations`](../reference/simwild.md#max_iterations) (80). The target is
[`stop_energy`](../reference/simwild.md#stop_energy) (default 10, against 100 in TetWild), unless an
entry of [`quality_field`](../reference/simwild.md#quality_field) matches the cell: a list of
`{"tags": expression, "quality": ...}`, where the last match wins. The output's `quality_target`
field holds each cell's target, and `quality` its energy.

On the curves, a stricter target inside the curves than in the background:

```python
--8<-- "docs/examples/python/simwild.py:quality"
```

| `quality_field` | triangles | largest energy inside the curves | in the background |
|---|---|---|---|
| none | 1,880 to 2,200 | 5.9 to 7.1 | 4.2 to 7.4 |
| `!_`: 4 | 2,120 to 2,200 | 3.0 to 3.3 | 3.3 to 3.5 |

The operations do not stop at a region's border, so the background improves with it. The other way
round, a higher target for `_` keeps the background from holding a run back
(`data/integration_tests/simwild_double_sphere_3d.json` sets it to 50); on the first run it makes
no difference beyond the run-to-run variation, since every tet is below 10 after the first
iteration anyway. A target the cells cannot reach makes the run refine until `max_iterations`:
`stop_energy` 5 on the spheres stalls at 5.9 with 75,000 tets after 20 iterations.

### Topology: `preserve_topology`

[`preserve_topology`](../reference/simwild.md#preserve_topology) (default `true`, unlike TetWild and
TriWild) keeps the topology of the interfaces as inserted: a collapse along them must pass a link
condition that keeps it, and the inputs are simplified after the insertion, within
[`eps_simplify_rel`](../reference/simwild.md#eps_simplify_rel). With `false`, the inputs are
simplified before the insertion, the collapses skip that check, and the interfaces' topology may
change. On
the curves at `length_rel` 0.02, `false` gives 1,842 triangles and a largest energy of 5.4 in one
run. On inputs with tiny features it matters more: see [Pitfalls](#pitfalls).

### Threads: `num_threads`

[`num_threads`](../reference/simwild.md#num_threads) 0 (the default) runs sequentially and gives
the same mesh every time. With threads, the result changes slightly from run to run, as the ranges
above show.

## The envelope a run returns

A run that optimizes also returns the surface it held the interfaces to: `outputs["envelope"]` in
Python, `outputs.at("envelope")` in C++, a triangle mesh in 3D and a set of segments in 2D. It is
the inputs as simwild simplified them, within
[`eps_simplify_rel`](../reference/simwild.md#eps_simplify_rel): in 3D, by default, the interfaces
right after the simplification that follows the insertion; with
[`skip_simplify`](../reference/simwild.md#skip_simplify), the inputs themselves; in 2D, the input
curves (simplified when `preserve_topology` is off).

A later `remeshing` of the same mesh takes it as the `envelope` input:

```python
--8<-- "docs/examples/python/simwild.py:envelope"
```

The finer remeshing gives 38,609 tets in one run. With the envelope, a remeshing holds the
interfaces to it; without it, simwild builds an envelope from the interfaces of the mesh it is
given, wherever the previous run left them. Pass it on when you remesh a mesh whose regions have
not changed since the run that returned it. In 3D, the remeshing first
simplifies the mesh within `eps_simplify_rel` of that envelope (unless
[`skip_simplify`](../reference/simwild.md#skip_simplify)) and then holds the optimization to the
simplified interfaces, so the envelope bounds how far the simplification moves them; with
`skip_simplify`, and in 2D, it bounds the whole run.

Do not pass it after a region edit. The edits change which faces are interfaces, so they return no
envelope, and an old one does not cover the new interfaces: simwild finds faces outside it, warns
`Some surface faces are outside the envelope. Re-build envelope`, and builds one from the regions.

On files, the `.msh` simwild writes holds the mesh only.
[`wm.save`](../python-api.md#meshes-and-files) writes the envelope next to it (`stem.msh` and
`stem_envelope.msh`) and `wm.load` reads both back. A `.msh`
may also carry an envelope as a group of triangles (lines in 2D) named `EnvelopeSurface`, as
`double_sphere.msh` and `topo_annots_groups.msh` do; simwild reads it as the envelope.

## Editing regions: `operation`

With a tagged mesh as input (a `.msh` file, or a mesh in memory with a `tags` attribute),
[`operation`](../reference/simwild.md#operation) chooses what to do: `remeshing` (the default)
improves the mesh; the others change which regions the cells belong to. The edits run in a
fraction of a second and do not optimize: `tight_seal_topo` and `resolve_overlaps` split edges and
leave poor elements, which a `remeshing` afterwards removes. In C++, an edit followed by a
remeshing:

```cpp
--8<-- "docs/examples/cpp/simwild.cpp:edit"
```

The examples below use three meshes of the example data: `double_sphere.msh`, two overlapping
spheres with regions `tag_0` and `tag_1` (15,131 tets: 1,479 in `tag_0` alone, 1,416 in `tag_1`
alone, 1,311 in both); `double_sphere_hole.msh`, the same spheres with the overlap left empty; and
`topo_annots_groups.msh`, a 2D label image, five regions on a 50 by 50 grid (5,000 triangles: 976
in `tag_0`, 3,102 in `tag_1`, 390 in `tag_2`, 142 in `tag_3`, 38 in `tag_4`, 352 in none). The
Python sections are in one script, `docs/examples/python/simwild_operations.py`.

### `remeshing`

The same optimization as after meshing, on the mesh as given, with
[`sizing_field`](../reference/simwild.md#sizing_field) and
[`quality_field`](../reference/simwild.md#quality_field) applying to it. Here the overlap of the
two spheres gets a finer edge length:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_remeshing.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild_operations.py:remeshing"
    ```

The mesh grows from 15,131 to about 46,000 tets, 26,000 of them in `tag_0 & tag_1`; the largest
energy is 7.6. Without the sizing field the same run gives about 17,000 tets. The regions stay
where they were, within the envelope; the cells change.

`double_sphere.msh` carries an envelope, but its interfaces are not all inside it, so this run logs
the warning above and builds one from the regions. `eps_rel` and `length_rel` are relative to the
diagonal of the given mesh.

### `fill_holes_topo`

For each entry of [`fill_holes_tags`](../reference/simwild.md#fill_holes_tags), a hole is a
connected set of cells outside that region, enclosed by it: it does not reach the mesh's boundary.
Each hole joins the region; its cells keep the regions they had.
[`fill_holes_threshold`](../reference/simwild.md#fill_holes_threshold), when positive, fills only
the holes of smaller area (volume in 3D), in model units.

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_fill_holes.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild_operations.py:fill_holes"
    ```

`tag_1` has two holes (`2 holes filled for tags {1}` in the log): one is a piece of `tag_0`, the
other a piece of `tag_2` with a rim of `tag_0`. Their 190 triangles join `tag_1` and keep their
regions:

```text
{'ambient': 352, 'tag_0': 846, 'tag_0 & tag_1': 130, 'tag_1': 3102, 'tag_1 & tag_2': 60,
 'tag_2': 330, 'tag_3': 142, 'tag_4': 38}
```

To give the hole to the enclosing region alone, follow with `tag_priority` or `replace_tags`.

### `tight_seal_topo`

Closes the gaps between two regions. Each entry of
[`tight_seal_tag_sets`](../reference/simwild.md#tight_seal_tag_sets) is a pair; a gap is a
connected set of cells in neither, enclosed (it does not reach the mesh's boundary) and touching
both. Each gap is split along the surface halfway between the two regions' boundaries: edges that
cross it are split, and every cell joins the nearer region, keeping the regions it had.
[`tight_seal_threshold`](../reference/simwild.md#tight_seal_threshold), when positive, seals only
the gaps of smaller area (volume in 3D).

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_tight_seal.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild_operations.py:tight_seal"
    ```

The lens between the two spheres, 1,311 tets of background, is the one gap:

```text
{'ambient': 10925, 'tag_0': 2770, 'tag_1': 2587}
```

The splits bring the mesh to 16,282 tets, and the cells they cut are poor: the largest energy is
2,112. Remesh afterwards.

### `keep_lcc`

Keeps the [`keep_lcc_num`](../reference/simwild.md#keep_lcc_num) largest connected components (by
area, volume in 3D) of each entry of [`keep_lcc_tags`](../reference/simwild.md#keep_lcc_tags), and
removes the region from the cells of the others.

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_keep_lcc.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild_operations.py:keep_lcc"
    ```

`tag_0` has four components. The largest has 816 triangles; the other three, 160 triangles, lose
`tag_0`, and since they had no other region they become background (`ambient` grows from 352 to
512). With `keep_lcc_num` 2, 908 triangles stay.

### `resolve_overlaps`

Splits overlaps between two regions. Each entry of
[`resolve_overlaps_tags`](../reference/simwild.md#resolve_overlaps_tags) is a pair of expressions;
the cells in both lose them, and are then split between the two as `tight_seal_topo` splits a gap:
along the surface halfway between the two regions' boundaries, each cell joining the nearer one.

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_resolve_overlaps.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild_operations.py:resolve_overlaps"
    ```

The 1,311 tets of the overlap are split between the two spheres: 2,770 tets in `tag_0`, 2,587 in
`tag_1`, none in both, 16,282 tets in all; the log ends with `Tags are intersection free.` It is the
same result as sealing the empty lens above. Remeshing it brings the largest energy from 2,112 to
9.0, with 16,835 tets:

```python
--8<-- "docs/examples/python/simwild_operations.py:resolve_then_remesh"
```

### `replace_tags`

Renames regions, or the overlap of several. For each position \(i\), the cells in every region of
[`replace_tags_in`](../reference/simwild.md#replace_tags_in)\([i]\) lose them and get those of
[`replace_tags_out`](../reference/simwild.md#replace_tags_out)\([i]\), which may be new. Here the
overlap becomes a region of its own:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_replace_tags.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild_operations.py:replace_tags"
    ```

```text
{'ambient': 10925, 'overlap': 1311, 'tag_0': 1479, 'tag_1': 1416}
```

The cells do not change, only their regions; `"replace_tags_out": ["tag_1"]` would give the overlap
to `tag_1`.

### `tag_priority`

Settles every overlap at once by rank: a cell keeps the first region of
[`tag_priority`](../reference/simwild.md#tag_priority) it belongs to and loses the ones after it in
the list. Regions not in the list are left alone.

=== "Command line"

    ```json
    --8<-- "docs/examples/json/simwild_tag_priority.json"
    ```

=== "Python"

    ```python
    --8<-- "docs/examples/python/simwild_operations.py:tag_priority"
    ```

The overlap goes to `tag_1`: 2,727 tets in `tag_1`, 1,479 in `tag_0`. Unlike `resolve_overlaps`,
which cuts the overlap in two along the middle, the interface stays where `tag_1`'s sphere was, and
no cell changes.

## Outputs

- The mesh: `output` on the command line (default `out`, written as `out.msh`), `outputs["mesh"]`
  in memory. Its tets (triangles in 2D) carry `tags` (the regions, and `ambient`), `quality` (the
  energy), `quality_target` and `quality_rel` (their ratio, at most 1 when the run converged); its
  vertices carry `sizing_field`.
- In memory, also `outputs["surface"]`: the interface triangles (in 2D, segments), and in 3D
  `outputs["edges"]`: the edges where the interfaces are not manifold, such as the circle where
  the two spheres cross. With [`write_vtu`](../reference/simwild.md#write_vtu), the file route
  writes VTU files of the same.
- `outputs["envelope"]`, see [above](#the-envelope-a-run-returns).
- With [`report`](../reference/simwild.md#report), a JSON file (in memory, the report) with the
  number of cells `#t` and vertices `#v`, `max_energy` and `avg_energy` over all cells, `eps`,
  `threads`, `time` and `all_rounded`.
- In 3D, with [`history_output`](../reference/simwild.md#history_output), the history of the
  interfaces, from which [tracking](tracking.md) maps points between the input and the output.

## Pitfalls

- **A misspelled option is ignored.** `simwild` runs with the default of an option it does not
  know; `wmtk_app` and the Python bindings log a warning (`simwild has no option ...`). The Python
  bindings also log it for `envelope`, which simwild does read (see
  [above](#the-envelope-a-run-returns)).
- **`ambient` in an expression.** It selects nothing; the background is `_`.
- **Open inputs.** Inside and outside come from winding numbers, which are not defined for an open
  curve or surface. `data/integration_tests/simwild_curves_2d.json` meshes `triwild_polylines.obj`,
  a drawing made of 54 open polylines: the regions come out, but not all their borders lie on the
  curves, and simwild warns `Some surface edges are outside the envelope. Rebuilding envelope from
  face tags.` The config turns `preserve_topology` off; with it on, the drawing's tiny features
  cost 4,758 and 7,867 triangles in two runs, against 336 and 364 with it off. Close the curves,
  or use `input_type` `surface` in 3D.
- **A tagged mesh in memory has a `tags` attribute.** A `.msh` that stores a label image instead
  (`tag_0` holding region numbers, as `bunny2d.msh` does) is converted on the file route; in memory
  it fails with `Unknown mesh attribute: tag_0`.
- **Region edits leave poor elements and no envelope.** Run a `remeshing` after `tight_seal_topo` or
  `resolve_overlaps`, and do not hand it an envelope from before the edit.
- **Unreachable targets.** A `stop_energy` or `quality_field` target below what the cells can reach
  refines until `max_iterations` and multiplies the mesh.

## See also

- [Reference: every option of `simwild`](../reference/simwild.md)
- [Regions](../concepts/regions.md): the `tags` format and the expression language
- [The multi-region workflow](workflows/regions.md): from surfaces to an offset region, in memory
- [TetWild](tetwild.md), [TriWild](triwild.md), [topological offset](topological_offset.md),
  [manifold extraction](manifold_extraction.md)
