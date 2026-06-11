# Topological Offsets
This file describes the basic usage modes of the topological_offsets component.

## Defining input
* **Single body:** Provide ```offset_selection="tag_0"``` (or whatever the desired tag name is). The boundary of that tag is used to create the offset. ```offset_in=true``` creates an offset in the inward direction. ```offset_out=true``` creates an offset in the outward direction. One or both can be true.
* **Boolean expression:** Provide a general boolean expression (ie ```offset_selection="(tag_0 | tag_1) & _```). The input simplicial complex consists of all simplices satisfying the boolean expression, around which the offset is created.

## Offset region tags
Once the offset is created, three parameters control what tags exist in the offset region: ```offset_output_tags```, ```overwrite_tags```, and ```protected_tags```:
* ```offset_output_tags```: a set of tags, ie ```["tag_0", "tag_3"]```. These tags are assigned to all tets in the offset region.
* ```overwrite_tags```: boolean, if ```true``` then all existing tags in the offset region are overwritten by ```offset_output_tag```. If ```false```, ```offset_output_tag``` is added to the existing tag set in each tet in the offset region.
* ```protected_tags```: a set of tags, ie ```["tag_1", "tag_2"]```. Only relevant if ```overwrite_tags=true```. No tags in this set are overwritten by ```offset_output_tag```.

## Respected topologies
Once the input simplicial complex is initialized, the offset is initialized around it. The topology of the offset region is *always* respected during offset growth toward the target distance. The boolean parameter ```respect_all_topologies``` controls whether the topology (after offset initialization) of all other tags is respected, *as if* the offset was to overwrite all other tags.
