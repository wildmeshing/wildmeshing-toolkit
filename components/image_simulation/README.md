# Image Simulation

This component provides multiple tools (called operations in the JSON) to generate and modify tet meshes.

## 1. Mesh Generation from Surfaces

The entry point is the tet mesh generation from surface meshes.

[Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/image_simulation_double_sphere_notop_3d.json)

## 2. Remeshing

This component always produces a .msh file as output. This file can be used as input for the next operation.
If the result of the initial meshing is not satisfactory, one can perform further remeshing. Remeshing is the default operation and does not need to be explicitly specified in the JSON.

## 3. Replace Tags

Any input surface is assigned a volumetric tag. These tags may fulfill different purposes. With the `"replace_tags"` operation, one can replace tags (or intersections of tags) with new ones.

[Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/image_simulation_replace_tags_3d.json)
