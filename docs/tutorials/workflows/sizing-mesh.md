# Edge length that varies: a sizing mesh

The meshing and remeshing components aim for one edge length everywhere:
[`length_rel`](../../reference/tetwild.md#length_rel) times the input's bounding box diagonal, or
[`length`](../../reference/tetwild.md#length) in the input's units. Only where the quality gets
stuck do they refine on their own. Often a simulation wants something else: small elements near a
crack, a load or a thin part, large ones far from it.

A **sizing mesh** gives the largest edge length as a field over space. TriWild, TetWild, SimWild
and isotropic remeshing take one as their `sizing_mesh` option:

| Component | The sizing mesh | Where a point reads it |
|---|---|---|
| [TriWild](../triwild.md) | triangles, in the plane | the triangle containing it |
| [TetWild](../tetwild.md) | tetrahedra | the tet containing it |
| [SimWild](../../reference/simwild.md#sizing_mesh) | triangles in 2D, tetrahedra in 3D | the cell containing it |
| [Isotropic remeshing](../isotropic_remeshing.md) | a triangle surface | the point of the surface closest to it |

The option is optional: without it, every component runs exactly as before.

## The sizing mesh

Any mesh with those cells whose vertices carry the length, in the input's units, as a vertex field
named `target_length`: one positive value per vertex, interpolated linearly inside the cells. It
does not have to match the input: a few large cells around it are enough for a smooth field. In
Python it is a [mesh dictionary](../../in-memory-io.md), here a square of two triangles whose
length grows from 1% of a circle's size on its left side to 6% on its right:

```python
--8<-- "docs/examples/python/workflow_sizing_mesh.py:build"
```

A point outside every cell is not covered, and takes the component's global length there. Make
the sizing mesh cover more than the input: TriWild and TetWild mesh a box larger than the input's
bounding box (by 1/15 of its diagonal on each side), and its vertices are sized too. A surface
sizing mesh covers every point: isotropic remeshing reads it at the closest point.

## TriWild

On the memory route, `sizing_mesh` names a mesh of the call. The inside of the circle, first at
the default `length_rel` 0.05, then with the sizing mesh. Each edge is compared with the field at
its midpoint:

=== "Python"

    ```python
    --8<-- "docs/examples/python/workflow_sizing_mesh.py:triwild"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/workflow_sizing_mesh.cpp:run"
    ```

```
triwild, length_rel 0.05: 289 triangles after 1 iteration(s), 52.9% of the edges past 4/3 of the field, median 1.38 of it
triwild, sizing mesh: 840 triangles after 1 iteration(s), 0.3% of the edges past 4/3 of the field, median 0.84 of it
```

<div>
<wmtk-viewer example="workflow_sizing_mesh_triwild"></wmtk-viewer>
</div>

`length_rel` 0.05 on the left, the sizing mesh on the right. Each triangle is colored by its
longest edge over the field at the edge's midpoint: white as long as the field, pale up to 4/3 of
it, red past that, blue shorter. The field grows from left to right; one length for the whole
circle suits its right side only. Scroll to zoom, switch the coloring below the meshes. Every
example is in the [gallery](../../gallery/index.md).

On the file route, `sizing_mesh` is the path of a mesh file, relative to
[`input_dir`](../../reference/triwild.md#input_dir) like the inputs. A `.msh` file written by
`wm.write_mesh` keeps the field, as a node field `target_length`:

```python
--8<-- "docs/examples/python/workflow_sizing_mesh.py:file"
```

```
triwild on files: 0.3% of the edges past 4/3 of the field, median 0.84 of it
```

The same mesh as on the memory route. On the command line, the same is
`"sizing_mesh": "sizing.msh"` in the JSON.

## A largest length, not a target

In TriWild, TetWild and SimWild every vertex carries a **sizing**, the factor of the target length its edges are split and
collapsed to (an edge longer than 4/3 of its target is split, one shorter than 4/5 collapsed;
[the optimization loop](../../concepts/quality.md#the-optimization-loop)). The sizing mesh sets
the sizing of every vertex it covers when the optimization starts, to the field's length over the
global one, and caps it after every pass:

- **New vertices follow the field.** A split's new vertex takes the mean of its ends' sizing, and
  the cap lowers it where the field asks for less.
- **Finer is allowed.** Where the quality gets stuck, the
  [stuck refinement](../../reference/tetwild.md#stuck_refine_factor) lowers the sizing further,
  and the cap only ever lowers it. A collapse that would make an element worse is still refused.
  Either way the mesh ends finer than the field there, and that is a valid result: the median
  edge above is 0.84 of the field.
- **Longer than `length` is allowed.** The field may ask for more than the global length;
  `length` and `length_rel` then only apply where it does not reach.

## Keeping the edges under the field

Two things hold the edges under the field, both only with a sizing mesh:

- **[`limit_collapse_length`](../../reference/triwild.md#limit_collapse_length)** is on unless set
  to false, in all four components. A collapse is judged by the edge it removes; without the limit, it may leave edges
  longer than their target, and since the optimization ends on collapses, nothing splits them
  again. With it, no collapse leaves an edge longer than 4/3 of its target, the length above which
  a split would cut it, as Botsch and Kobbelt's isotropic remeshing does.
- **The optimization does not stop on quality alone** (TriWild, TetWild, SimWild). It goes on past
  [`stop_energy`](../../reference/triwild.md#stop_energy) while more than 1% of the edges are longer
  than 4/3 of their target and that share still drops by a tenth from one iteration to the next, up
  to [`max_iterations`](../../reference/triwild.md#max_iterations). Edges shorter than the field
  never keep it going.

Turned off, the limit costs fewer elements and more edges past the field; the second rule then
runs one more iteration:

```python
--8<-- "docs/examples/python/workflow_sizing_mesh.py:limit"
```

```
limit_collapse_length True: 840 triangles after 1 iteration(s), 0.3% of the edges past 4/3 of the field, median 0.84 of it
limit_collapse_length False: 577 triangles after 2 iteration(s), 13.0% of the edges past 4/3 of the field, median 1.02 of it
```

<div>
<wmtk-viewer example="workflow_sizing_mesh_limit"></wmtk-viewer>
</div>

The limit on the left, off on the right: there, the collapses leave the red triangles, whose
longest edge is past the field.

Smoothing can still stretch an edge a little past 4/3 of its target: to keep every edge under a
bound, give about 3/4 of it as the field.

The field is read at the vertices: an edge's target is the mean of its ends'. A field smooth at the
scale of the edges, as above, is kept closely; one that varies a lot within an edge's length is
followed less well, since the field between the ends does not count: give the sizing mesh cells
no smaller than the edges you expect. The stuck refinement grades its own refinement into the
neighbourhood, but not past a vertex whose sizing the field set above the global length.

## TetWild

The sizing mesh is a tet mesh. A box around a sphere of radius 0.7, as five tets, whose length
grows from 0.03 at its bottom to 0.15 at its top:

```python
--8<-- "docs/examples/python/workflow_sizing_mesh.py:tetwild"
```

```
tetwild, length_rel 0.05: 12687 tets, 29.6% of the edges past 4/3 of the field, median 1.16 of it
tetwild, sizing mesh: 34011 tets, 0.6% of the edges past 4/3 of the field, median 0.85 of it
```

<div>
<wmtk-viewer example="workflow_sizing_mesh_tetwild"></wmtk-viewer>
</div>

The tets the cut exposes are colored as above, the faces of the boundary gray. Here z points to
the left: the field grows from right to left. Drag to rotate, move the cut with the slider.

## Isotropic remeshing

The sizing mesh is a surface, and isotropic remeshing reads it at the point closest to each
vertex, again wherever an operation moves one; an edge's target is the mean of its ends'. For
isotropic remeshing the field is a target rather than a largest length, as `length` is: it splits
and collapses every edge toward it, for a fixed number of iterations, with no stuck refinement and
no energy target to stop at. The input itself can be the sizing mesh: here the bunny, with a field
that goes from 4% of its diagonal at its feet to 1% at its ears:

```python
--8<-- "docs/examples/python/workflow_sizing_mesh.py:isotropic"
```

```
isotropic_remeshing, length_rel 0.025: 2559 triangles, 21.0% of the edges past 4/3 of the field, median 0.96 of it
isotropic_remeshing, sizing mesh: 3633 triangles, 0.5% of the edges past 4/3 of the field, median 1.02 of it
```

<div>
<wmtk-viewer example="workflow_sizing_mesh_isotropic"></wmtk-viewer>
</div>

At one length, the triangles are too long for the ears (red); with the sizing mesh, they follow
the field from the feet to the ears.

## SimWild: with regions

SimWild also sizes by region:
[`sizing_field`](../../reference/simwild.md#sizing_field) gives a length per region expression.
With both, a vertex takes the smaller of its regions' length and the sizing mesh's; where only
the sizing mesh reaches, its length, even longer than `length`; where only a region, the region's.
Only the [`remeshing`](../../reference/simwild.md#operation) operation optimizes: the region
operations ignore both options, so one set of options can be passed to every step of a pipeline.

## The coarsening pass

The optional [coarsening pass](../../reference/tetwild.md#coarsen_pass) of TriWild, TetWild and
SimWild runs after the optimization and removes the elements the quality target does not need: it
takes collapses the optimization refused, and keeps one when, after smoothing around it, no
element there is worse than before ([the coarsening pass](../../concepts/quality.md#the-coarsening-pass)).
With a sizing mesh, `limit_collapse_length` holds it under the field too: none of its collapses
leaves an edge longer than 4/3 of the field's length. It coarsens where it can without breaking
the field:

```python
--8<-- "docs/examples/python/workflow_sizing_mesh.py:coarsen"
```

```
tetwild, coarsen_pass: 32667 tets, 0.6% of the edges past 4/3 of the field, median 0.87 of it
tetwild, coarsen_pass, limit off: 15009 tets, 24.0% of the edges past 4/3 of the field, median 1.10 of it
```

<div>
<wmtk-viewer example="workflow_sizing_mesh_coarsen"></wmtk-viewer>
</div>

With the limit (on the left), the pass removes 4% of the 34011 tets, and the field still holds:
the optimization had already brought the edges to the field, so most collapses would make them
longer than it. Without the limit, it removes 56% of them, and makes the edges longer than the
field (red). Where the mesh is finer than the field, after a stuck refinement or near the input's
details, the pass has more to remove.

[Unbounded](../../reference/tetwild.md#coarsen_unbounded) (the default), the pass knows no target
length but the limit's. Bounded, it also leaves alone the edges at or past 4/5 of the target: with
a sizing mesh, of the field's length at the edge's ends, also where that is longer than `length`.

## Summary

For TriWild, TetWild and SimWild; isotropic remeshing shares the first and the
`limit_collapse_length` rows.

| | Without a sizing mesh | With one |
|---|---|---|
| Largest edge length | `length`, or `length_rel` times the diagonal | the field's where it covers the vertex, else as without |
| Finer than that | stuck refinement, and collapses refused on quality | the same |
| `limit_collapse_length` | off unless set | on unless set to false |
| Stops when | the energy is under `stop_energy` | the same, and at most 1% of the edges past 4/3 of their target, or no more progress |
| Coarsening pass | as far as the energy allows; bounded, stops at 4/5 of `length` | under 4/3 of the field (the limit); bounded, also stops at 4/5 of it |
| SimWild's `sizing_field` | the region's length | the smaller of the region's and the field's |
