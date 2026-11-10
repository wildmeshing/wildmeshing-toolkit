# Topological Offsets
This file describes the basic usage modes of the topological_offsets component.

## Defining input
Each entry of the list ```offsets``` is one offset: ```{"selection": ..., "output_tags": [...], "target_distance": ...}```. Several entries in one run are independent offsets, each with its own band, output tags and distance, constructed in one march; entries that touch are refused.
* **Single body:** Provide ```offsets=[{"selection": "tag_0"}]``` (or whatever the desired tag name is). The boundary of that tag is used to create the offset. ```offset_in=true``` creates an offset in the inward direction. ```offset_out=true``` creates an offset in the outward direction. One or both can be true.
* **Boolean expression:** Provide a general boolean expression as the selection (ie ```"selection": "(tag_0 | tag_1) & ambient"```). The input simplicial complex consists of all simplices satisfying the boolean expression, around which the offset is created. The 'ambient' tag should be specified as ```ambient``` and not ```_```.
* **Open curve / sheet:** The .msh's ```EnvelopeSurface``` group (line elements in 2D, triangles in 3D) is registered as a tag under its own name, so naming it as an entry's ```selection``` makes that curve or sheet the complex and the band grows on both of its sides.

## Offset region tags
Once the offset is created, two parameters control what tags exist in the offset region: an entry's ```output_tags```, and ```protected_tags```. The tags in ```output_tags``` (e.g. ```["tag_0", "tag_3"]```) are assigned to all elements in that entry's offset. All existing tags are overwritten. ```protected_tags``` is a tag expression (e.g. ```"tag_1 | tag_2"```; empty: nothing): the march leaves the cells whose tags satisfy it as they are, neither splitting them nor taking them into its band, and a face the band covers whose own tags satisfy it keeps the tags it names. To write the ```'ambient'``` tag, set ```"output_tags": []```. The ```'ambient'``` tag itself cannot be protected, it is always overwritten.

## Respected topologies
Once the input simplicial complex is initialized, the offset is initialized around it. The topology of the offset region is *always* respected during offset growth toward the target distance.

## One pipeline, two dimensions
The 2D (```TopoOffsetTriMesh```) and 3D (```TopoOffsetTetMesh```) meshes run the same algorithm one dimension apart, and are kept in lock step file for file: ```TopoOffsetTriMesh.cpp``` / ```TopoOffsetTetMesh.cpp``` (construction and I/O), ```Optimize2d.cpp``` / ```Optimize3d.cpp``` (the optimization loop, criteria and diagnostics), ```FrontSmooth2d.cpp``` / ```FrontSmooth3d.cpp``` (the front placement), with the dimension-specific operation hooks in ```EdgeSplittingTri.cpp``` and ```EdgeSplittingTet.cpp``` / ```Collapse.cpp``` / ```Swap.cpp``` / ```Smooth.cpp```. Every key in ```topological_offset_spec.json``` means the same thing in both, and the console logs are the same lines (edges in 2D, faces in 3D). A stage of the run, in order:

1. Construction, on the input mesh AS GIVEN: simplicial embedding, marching (```construction_mode```: sphere tracing to the target, or when the target is not below the maximum marchable distance to half of it / edge midpoints), the band one cell thick. There is no pre-optimization pass -- the input's quality and resolution decide the constructed offset directly, and supplying a mesh good enough for that is the caller's job.
2. The smooth offset potential (or the euclidean distance) over the complex as loaded, one field per connected piece of the complex.
3. The single-phase loop, up to ```max_rounds``` turns of split / collapse / swap, each followed by ```interleaved_smoothing_passes``` smoothing passes in which front vertices are placed by the shared smoother's free solve on AMIPS plus the offset term. One measure and one bar throughout: the RMS relative error of the field over a face's ```stencil_order``` stencil, against ```front_conv```. Faces over it are refined by halving their corners' sizing scalars.
4. A finishing TriWild / TetWild pass with the front frozen if max AMIPS is still over ```stop_energy```.
