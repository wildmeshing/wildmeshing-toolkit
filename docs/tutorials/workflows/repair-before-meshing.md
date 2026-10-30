# Repair before meshing

A tagged mesh can have regions that touch only along an edge or at a vertex: two parts whose
overlap was removed, two segments of a scan that meet at a point. The union of such regions has a
non-manifold boundary, and a quality remeshing keeps it: [simwild](../simwild.md)'s remeshing
preserves the regions' topology, so it improves the cells around the contact without removing it.
The repair comes first: [manifold extraction](../manifold_extraction.md) separates (or joins) the
regions at the contact, and the remeshing then restores the quality that the cut costs.

The example is `double_sphere_hole.msh`: two spheres, `tag_0` and `tag_1`, whose overlap was
removed, so that they touch along a circle. The boundary of `tag_0 | tag_1` has 970 triangles and
28 non-manifold edges.

<div>
<wmtk-viewer example="workflow_repair"></wmtk-viewer>
</div>

The input on the left, the result on the right: drag to rotate, scroll to zoom; on the tet mesh, move the cut with the slider. Every example is in the [gallery](../../gallery/index.md).

## The chain

=== "Command line"

    Two files, run in order; the second reads what the first wrote.

    ```json
    --8<-- "docs/examples/json/workflow_repair/1_manifold_extraction.json"
    ```

    ```json
    --8<-- "docs/examples/json/workflow_repair/2_remeshing.json"
    ```

    ```bash
    wmtk_app -j 1_manifold_extraction.json
    wmtk_app -j 2_remeshing.json
    ```

    The result is `spheres_remeshed.msh`.

=== "Python"

    ```python
    --8<-- "docs/examples/python/workflow_repair.py:chain"
    ```

    The script also defines `nonmanifold_edges`, which counts the edges of a union's boundary that
    are not on exactly two of its triangles.

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/workflow_repair.cpp:chain"
    ```

    Link `wmtk::manifold_extraction` and `wmtk::simwild`.

Each step, measured on the union `tag_0 | tag_1`:

| step | tets | non-manifold edges on the boundary | max AMIPS |
|---|---|---|---|
| input | 15,131 | 28 | 7.26 |
| 1. manifold extraction | 17,093 | 0 | 42.06 |
| 2. simwild remeshing | about 18,000 | 0 | below 10 |
| simwild remeshing alone, without step 1 | 16,991 | 28 | 6.95 |

1. **Manifold extraction** finds the 28 edges of the circle, carves one ring of cells around them
   and makes it background. The two spheres no longer touch: the union's boundary is two closed
   surfaces, 1,284 triangles. The cut is not optimized, and the cells next to it are thin.
2. **Simwild remeshing** improves the cells until every one is below its
   [`stop_energy`](../../reference/simwild.md#stop_energy) (10 by default), holding each region's
   surface within [`eps_rel`](../../reference/simwild.md#eps_rel) of where it was. It keeps the
   regions' topology, so the boundary stays manifold. With four threads the tet count varies from
   run to run: 17,870 to 18,801 in six runs, with a max energy of 9.81 to 9.98.

Remeshing alone, the last row, gives better cells and the same 28 non-manifold edges.

## Choices

- **Separate or join.** `fill_tags` `[]` separates the regions at the contact; a tag of the
  selection, such as `["tag_0"]`, joins them with a tube of that tag, and the removed overlap
  becomes a closed cavity. See [what fills the cut](../manifold_extraction.md#what-fills-the-cut-fill_tags).
- **The width of the cut** is at most one ring of cells around the contact, whatever `radius_rel`
  asks (see [the size of the cut](../manifold_extraction.md#the-size-of-the-cut-radius_rel-radius)).
  A thinner cut leaves a thinner gap for the remeshing to resolve: with `radius_rel` 0.005
  (max AMIPS 112.9 after the cut), the remeshed mesh has 23,669 tets and a boundary of 2,158
  triangles, against about 18,000 and 1,338 at the default.
- **The boundary as a surface.** After the remeshing, simwild's `surface` output (in memory) is
  the regions' boundary, 1,338 triangles in one run, manifold, for a solver or a surface tool.

## Pitfalls

- **Check that there is something to repair.** Manifold extraction returns a selection whose
  boundary is already manifold unchanged; count first, as the Python example does, to know
  whether the step did anything.
- **The selection decides.** Each sphere alone is manifold; only the union is not. Select the
  regions whose common boundary has to be manifold.

## See also

- [Manifold extraction](../manifold_extraction.md)
- [SimWild](../simwild.md), and its [reference](../../reference/simwild.md)
- [Topological offset](../topological_offset.md): the construction manifold extraction carves with
