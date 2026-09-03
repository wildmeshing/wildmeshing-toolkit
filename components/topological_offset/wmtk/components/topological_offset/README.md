# Topological Offsets
This file describes the basic usage modes of the topological_offsets component.

## Defining input
* **Single body:** Provide ```offset_selection="tag_0"``` (or whatever the desired tag name is). The boundary of that tag is used to create the offset. ```offset_in=true``` creates an offset in the inward direction. ```offset_out=true``` creates an offset in the outward direction. One or both can be true.
* **Boolean expression:** Provide a general boolean expression (ie ```offset_selection="(tag_0 | tag_1) & ambient"```). The input simplicial complex consists of all simplices satisfying the boolean expression, around which the offset is created. The 'ambient' tag should be specified as ```ambient``` and not ```_```.
* **Open curve / sheet:** The .msh's ```EnvelopeSurface``` group (line elements in 2D, triangles in 3D) is registered as a tag under its own name, so naming it in ```offset_selection``` makes that curve or sheet the complex and the band grows on both of its sides.

## Offset region tags
Once the offset is created, two parameters control what tags exist in the offset region: ```offset_output_tags```, and ```protected_tags```. The tags in ```offset_output_tags``` (e.g. ```["tag_0", "tag_3"]```) are assigned to all elements in the created offset. All existing tags are overwritten, except for those in ```protected_tags``` (e.g. ```["tag_1", "tag_2"]```), which are never overwritten. To write the ```'ambient'``` tag, set ```offset_output_tags=[]```. The ```'ambient'``` tag itself cannot be protected, it is always overwritten.

## Respected topologies
Once the input simplicial complex is initialized, the offset is initialized around it. The topology of the offset region is *always* respected during offset growth toward the target distance.

## One pipeline, two dimensions
The 2D (```TopoOffsetTriMesh```) and 3D (```TopoOffsetTetMesh```) meshes run the same algorithm one dimension apart, and are kept in lock step file for file: ```TopoOffsetTriMesh.cpp``` / ```TopoOffsetTetMesh.cpp``` (construction and I/O), ```Optimize2d.cpp``` / ```Optimize3d.cpp``` (the optimization loop, criteria and diagnostics), ```FrontSmooth2d.cpp``` / ```FrontSmooth3d.cpp``` (the front placement), with the dimension-specific operation hooks in ```EdgeSplittingTri.cpp``` and ```EdgeSplittingTet.cpp``` / ```Collapse.cpp``` / ```Swap.cpp``` / ```Smooth.cpp```. Every key in ```topological_offset_spec.json``` means the same thing in both, and the console logs are the same lines (edges in 2D, faces in 3D). A stage of the run, in order:

1. ```pre_optimize_input```: TriWild / TetWild over the input mesh, held by the per-tag region envelopes, with the sizing field seeded to ```target_distance``` on the input-complex boundary.
2. Construction: simplicial embedding, marching at edge midpoints, the band one cell thick.
3. The smooth offset potential (or the euclidean distance) over the complex as loaded, one field per connected piece of the complex.
4. The single-phase loop, up to ```max_rounds``` turns of split / collapse / swap, each followed by ```interleaved_smoothing_passes``` smoothing passes in which front vertices are placed by a one-dimensional solve along the field normal (```front_normal_projection```) against the offset and alignment terms; after each turn every front vertex is classified as placed / travelling / pressed / stuck against ```front_conv_rel``` (```front_conv_criterion```), and chords sagging over the accuracy are refined (```refine_front_from_sag```).
5. A finishing TriWild / TetWild pass with the front frozen if max AMIPS is still over ```stop_energy```.
