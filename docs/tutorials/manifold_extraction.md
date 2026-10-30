# Manifold extraction

`manifold_extraction` makes the boundary of a selection of regions manifold. In a tagged tet mesh
(3D) or triangle mesh (2D), a selection is a set of cells given by a tag expression, and its
boundary is the surface (3D) or curve (2D) between the selected cells and the others. Where two
parts of the selection touch only along an edge or at a vertex, that boundary is non-manifold
there. The component finds those edges and vertices, carves a thin region around them, and gives
the carved cells the tags in `fill_tags`: the background separates the parts, a selected tag joins
them.

Use it on a tagged mesh whose regions touch, before a step that needs a manifold boundary, such
as a surface extracted for a solver or a surface remesher. The carving is a
[topological offset](topological_offset.md) of the non-manifold edges and vertices, construction
only: it is fast, but the cells it cuts are of lower quality, and
[simwild remeshing](simwild.md) is the step after it (see
[Repair before meshing](workflows/repair-before-meshing.md)).

## What is non-manifold

The test is local, at each vertex and edge on the selection's boundary:

- **An edge (3D)** is non-manifold when the selected tets around it do not form one fan: going
  from tet to tet across their faces around the edge does not reach all of them. Around an edge
  of the circle where two balls touch, the tets go ball, outside, ball, outside: the selected ones
  form two fans.
- **A vertex (3D)** is non-manifold when the selected tets around it, or the other tets around it,
  are not one face-connected set each. The outside of the mesh counts as one cell of the others.
- **A vertex (2D)** is non-manifold when the selected triangles around it do not form one fan
  across their edges: two triangles that share a vertex and no edge.

An edge or vertex that is not on the boundary is not tested. A boundary that passes the test
everywhere is a closed manifold surface (in 2D, closed curves).

## A first run

Two spheres, `tag_0` and `tag_1`, overlap; the overlap was removed, so they touch along the
circle where their surfaces cross. The union `tag_0 | tag_1` has a non-manifold boundary along
that circle. With the default `fill_tags`, the background:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/manifold_extraction.json"
    ```

    ```bash
    wmtk_app -j manifold_extraction.json
    ```

    The input must be a `.msh`. The repaired mesh is `double_sphere_manifold.msh`, and with
    `write_surface` the boundary of the selection is `double_sphere_manifold.obj`.

=== "Python"

    ```python
    --8<-- "docs/examples/python/manifold_extraction.py:memory"
    ```

    On files instead, with the same options as the JSON file:

    ```python
    --8<-- "docs/examples/python/manifold_extraction.py:file"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/manifold_extraction.cpp:run"
    ```

    Link `wmtk::manifold_extraction`.

The log reports 28 non-manifold edges and 28 non-manifold vertices (the count of vertices includes
the edges' ends), and the mesh goes from 15,131 to 17,093 tets in a fraction of a second. The
union's boundary had 970 triangles and 28 non-manifold edges; it now has 1,284 triangles, no
non-manifold edge or vertex, and is two closed surfaces: the spheres no longer touch. To check a
result, count the edges of the boundary that are not on exactly two triangles:

```python
--8<-- "docs/examples/python/manifold_extraction.py:check"
```

It prints `28 -> 0`, and `0` for the extracted surface.

### In 2D

Two triangles of `tag_0` share one vertex and no edge (`vertex_adj_tris.msh`, 50 triangles):

=== "Command line"

    ```json
    --8<-- "docs/examples/json/manifold_extraction_2d.json"
    ```

    ```bash
    wmtk_app -j manifold_extraction_2d.json
    ```

    With `write_surface` in 2D, `vertex_adj_tris_manifold.obj` holds the boundary as polylines
    (`l` records).

=== "Python"

    ```python
    --8<-- "docs/examples/python/manifold_extraction.py:planar"
    ```

The log reports one non-manifold vertex. The mesh goes from 50 to 62 triangles; `tag_0` is four
triangles in two pieces, which no longer touch, with a manifold boundary of 8 edges.

<div>
<wmtk-viewer example="manifold_extraction"></wmtk-viewer>
</div>

The input on the left, the result on the right: drag to rotate, scroll to zoom; on the tet mesh, move the cut with the slider. Every example is in the [gallery](../gallery/index.md).

## Key settings

### The selection: `tag_selection`

[`tag_selection`](../reference/manifold_extraction.md#tag_selection) (default `tag_0`) is the
selection whose boundary is made manifold: an expression over the tag names, with `&` (and), `|`
(or), `!` (not) and parentheses, as in simwild. `ambient` names the background. A cell is
selected when its tags satisfy the expression. On the two spheres, `tag_0 | tag_1` and
`!ambient` select the same cells and give the same mesh.

The selection decides what counts as touching. `tag_0` alone is a ball with a dent where the
overlap was; its boundary is manifold, and so is `tag_1`'s. Only their union is non-manifold.

### What fills the cut: `fill_tags`

[`fill_tags`](../reference/manifold_extraction.md#fill_tags) are the tags given to every carved
cell, replacing the tags it had. The default, `[]`, is the background: `ambient` cannot be named
here (the component ignores it, with a warning). On the two spheres, the carved tube is 886 tets:

```python
--8<-- "docs/examples/python/manifold_extraction.py:fill"
```

| `fill_tags` | `tag_0` tets | `tag_1` tets | boundary of the union |
|---|---|---|---|
| `[]` | 1,637 | 1,574 | 1,284 triangles, two closed surfaces |
| `["tag_0"]` | 2,523 | 1,574 | 1,568 triangles, an outer surface and the cavity |

With the background, the parts come apart. With a selected tag, the tube joins them, and the
removed overlap becomes a closed cavity. A tag the mesh does not have is added as a new region
(the log says `Tag 'bridge' does not exist. Adding to mesh.`), which marks the carved cells; for
the selection it is background unless the expression names it. In 2D, filling with `tag_0` gives
`tag_0` 10 triangles in one piece, with a manifold boundary of 10 edges.

### The size of the cut: `radius_rel`, `radius`

[`radius_rel`](../reference/manifold_extraction.md#radius_rel) (default 0.01) is the radius of the
carved region, relative to the bounding box diagonal;
[`radius`](../reference/manifold_extraction.md#radius), when positive, replaces it with an absolute
length. The carved region is always one ring of cells around the non-manifold edges and vertices;
the radius places the cut inside that ring. Each edge that leaves a non-manifold simplex is cut at
the radius when the far end of every such edge is farther than the radius from the non-manifold
simplices; otherwise every one is cut at half the smallest of those distances (the log calls it the
maximum marchable distance). The log says which:

```text
[construction] construction_mode max_marchable_fallback: maximum marchable distance 0.0587032 over 356 marched edges; target_distance 0.0674414 is not below it, so the march traces to half of it, 0.0293516 (0.4352x target_distance)
```

On the two spheres (diagonal 3.37), the default radius, 0.0337, is below that limit, 0.0587:

| `radius_rel` | where the cut is | max AMIPS |
|---|---|---|
| (input) | | 7.26 |
| 0.005 | at the radius, 0.0169 | 112.9 |
| 0.01 | at the radius, 0.0337 | 42.06 |
| 0.02 | at half the limit, 0.0294 | 51.43 |
| 0.05 | at half the limit, 0.0294 | 51.43 |

So a small radius cuts thin cells next to the non-manifold edges, and a radius above the limit
changes nothing: the cut never reaches beyond the cells that touch the contact.

### Output files: `write_surface`

[`write_surface`](../reference/manifold_extraction.md#write_surface) writes the boundary of the
selection after the repair as `<output>.obj`: triangles in 3D, polylines in 2D. The component checks
it before writing and logs `Vertex manifoldness check passed.` and `Edge manifoldness check
passed.` (in 2D, `Curve manifoldness check passed.`), or a warning.

## Outputs

- The repaired mesh: `output` on the command line (default `out`, written as `out.msh`),
  `outputs["mesh"]` in memory. It has the input's tags, the fill tags on the carved cells, and
  diagnostic fields of the offset (`labels`, `offset_tag`, `amips` and others).
- `outputs["surface"]` in memory: the boundary of the selection after the repair, a triangle mesh
  in 3D and an edge mesh in 2D, with vertices of its own. On files, `write_surface` writes it.
- With `report`, a JSON file with the vertex, edge, face (and in 3D tet) counts before and after,
  and the run time. In memory, the report is the second return value.
- With `save_vtu`, the mesh as `.vtu` files for ParaView.

## Pitfalls

- **A selection that is already manifold, in 3D, crashes.** When no edge or vertex is
  non-manifold, the 3D run stops with a segmentation fault instead of returning the mesh (on the
  two spheres, `tag_0` alone, or `tag_0 & tag_1`, which selects nothing). The 2D run returns the
  mesh unchanged. Check the selection first, as in the example above.
- **The quality drops.** The cut is not optimized: on the two spheres the max AMIPS goes from
  7.26 to 42.06 (avg 3.69 to 4.29). Run [simwild remeshing](simwild.md) on the result; it keeps
  the regions' topology and so their manifold boundary
  ([Repair before meshing](workflows/repair-before-meshing.md)).
- **The radius is not a distance guarantee.** See [the size of the cut](#the-size-of-the-cut-radius_rel-radius):
  the cut stays within one ring of cells.
- **A selection that reaches the mesh boundary.** Vertices on the mesh boundary are judged with the
  outside of the mesh as one more cell; the log warns `Vertex of offset input complex lies on mesh
  boundary`, and the final check may report a non-manifold surface, which it attributes to that.
  Keep the selection inside the domain, with background cells around it.
- **Face tags are refused.** A mesh with tagged faces (simwild's `input_type` `surface`) stops
  with `manifold_extraction does not take tagged faces yet`.

## See also

- [Reference: every option of `manifold_extraction`](../reference/manifold_extraction.md)
- [Topological offset](topological_offset.md): the carving is its construction
- [Repair before meshing](workflows/repair-before-meshing.md): manifold extraction, then simwild remeshing
- [SimWild](simwild.md): the tag expressions, and the remeshing that follows
