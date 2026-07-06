# Topological Offsets
This file describes the basic usage modes of the topological_offsets component.

## Defining input
* **Single body:** Provide ```offset_selection="tag_0"``` (or whatever the desired tag name is). The boundary of that tag is used to create the offset. ```offset_in=true``` creates an offset in the inward direction. ```offset_out=true``` creates an offset in the outward direction. One or both can be true.
* **Boolean expression:** Provide a general boolean expression (ie ```offset_selection="(tag_0 | tag_1) & ambient"```). The input simplicial complex consists of all simplices satisfying the boolean expression, around which the offset is created. The 'ambient' tag should be specified as ```ambient``` and not ```_```.

## Offset region tags
Once the offset is created, two parameters control what tags exist in the offset region: ```offset_output_tags```, and ```protected_tags```. The tags in ```offset_output_tags``` (e.g. ```["tag_0", "tag_3"]```) are assigned to all elements in the created offset. All existing tags are overwritten, except for those in ```protected_tags``` (e.g. ```["tag_1", "tag_2"]```), which are never overwritten. To write the ```'ambient'``` tag, set ```offset_output_tags=[]```. The ```'ambient'``` tag itself cannot be protected, it is always overwritten.

## Respected topologies
Once the input simplicial complex is initialized, the offset is initialized around it. The topology of the offset region is *always* respected during offset growth toward the target distance. The boolean parameter ```respect_all_topologies``` controls whether the topology (after offset initialization) of all other tags is respected, *as if* the offset was to overwrite all other tags.
