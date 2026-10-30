# Tutorials

One tutorial per component: what it does, a first run in JSON, Python and C++, the settings that
matter and what they change, the outputs, and the pitfalls. Every example on these pages is a
file in [`docs/examples`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/docs/examples)
that CI runs.

| Group | Component | For |
|---|---|---|
| Surface remeshing | [Shortest edge collapse](shortest_edge_collapse.md) | Fast simplification to a vertex budget |
| | [QSlim](qslim.md) | Simplification that keeps the shape (quadric error) |
| | [Isotropic remeshing](isotropic_remeshing.md) | Uniform, well-shaped triangles at a target edge length |
| Meshing | [TetWild](tetwild.md) | Tet meshes of imperfect surfaces |
| | [TriWild](triwild.md) | Triangle meshes of curve networks |
| Multi-region meshes | [SimWild](simwild.md) | Meshing several inputs into one mesh with regions, and editing the regions |
| Offsets and repair | [Topological offset](topological_offset.md) | A region at a distance around a selection |
| | [Manifold extraction](manifold_extraction.md) | A manifold boundary for a region |
| Mapping | [Tracking](tracking.md) | Correspondences between a remeshing's input and output |

[Workflows](workflows/index.md) chain components into pipelines.

`prismatic_mesh` and `c1_simplification` are work in progress: their options are in the
[reference](../reference/index.md), without a tutorial yet.

Before the tutorials, [Getting started](../getting-started/index.md) covers installation and the
two ways to call a component.
