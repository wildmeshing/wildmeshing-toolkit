# Workflows

Pipelines of several components, each as one script.

- [Multi-region pipeline](regions.md): surfaces into one tet mesh with a region per input
  (SimWild), its regions edited, then a topological offset around one of them.
- [TetWild with features](tetwild-features.md): feature curves and points forced into a tet
  mesh, and how to check they are kept.
- [Repair before meshing](repair-before-meshing.md): make a region's boundary manifold, then
  remesh.
- [Dense surface to tet mesh, with correspondences](../tracking.md): remeshing and TetWild with
  their histories, and a map from the dense input to the tet mesh's surface (the tracking
  tutorial).
