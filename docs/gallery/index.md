# Gallery

Each example runs a component on one of the example meshes, from the current code: CI runs them
again whenever the code they depend on changes (`docs/gallery/examples.yaml`,
`docs/tools/build_gallery.py`).

Drag to rotate, scroll to zoom, right-drag to pan. On a tet mesh, the cut hides the tets on one
side of a plane: pick its axis and move it with the slider. The faces the cut exposes are colored
by element quality (the AMIPS energy, on a log scale from the mesh's best to its worst element);
the faces of the mesh's boundary are gray.

<script type="module" src="../assets/viewer/wmtk-viewer.min.js"></script>

## TetWild on the Octocat

<div>
<wmtk-viewer input="assets/tetwild_octocat/input.wmv.gz"
             output="assets/tetwild_octocat/output.wmv.gz"
             labels="Input|TetWild" color="quality" cut="x"
             poster="assets/tetwild_octocat/poster.png"
             alt="The Octocat tetrahedralized by TetWild, cut open, with its input surface on the left"></wmtk-viewer>
</div>

The Octocat is a triangle soup with non-manifold welds. [TetWild](../tutorials/tetwild.md)
meshes its bounding box and keeps the tets inside the input by its winding number
(`filter: input`; the default, `none`, keeps the whole box):

```yaml
--8<-- "docs/gallery/examples.yaml:15:29"
```
