# SimWild

This component provides multiple tools (called operations in the JSON) to generate and modify tet meshes. Most functionality is also available in 2D, i.e., on triangular meshes.

## Python

Python wrappers for these operations, with an example for each, are in the [pywmtk](../../python/pywmtk/README.md) package. It also wraps topological offsets, manifold extraction and the PolyFEM-backed operations.

## Operations

All operations produce a .msh file as output. This file can be used as input for the next operation. The only exception is mesh generation, which takes surface or line meshes as input.

| Operation | Purpose | JSON example |
| --- | --- | --- |
| Mesh generation | Tet/tri mesh from surface/line meshes | [simwild_double_sphere_notop_3d](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_double_sphere_notop_3d.json) |
| `remeshing` | Improve the mesh quality after other operations | [simwild_remeshing_2d](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_remeshing_2d.json) |
| `replace_tags` | Replace tags (or intersections of tags) with new ones | [simwild_replace_tags_3d](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_replace_tags_3d.json) |
| `resolve_overlaps` | Split the overlap of two regions into the two regions | [simwild_resolve_overlaps_3d](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_resolve_overlaps_3d.json) |
| `tag_priority` | Keep only the highest-priority tag of each tet | — |
| `tight_seal_topo` | Close the holes between two regions | [simwild_tight_seal_3d](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_tight_seal_3d.json) |
| `keep_lcc` | Remove all but the largest connected component of a tag | [simwild_keep_lcc_2d](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_keep_lcc_2d.json) |
