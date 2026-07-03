# SimWild

This component provides multiple tools (called operations in the JSON) to generate and modify tet meshes. Most functionality is also available in 2D, i.e., on triangular meshes.

## Python Wrapper

For convenience, we provide a [Python wrapper](pysimwild/simwild/simwild.py) for the SimWild component. The wrapper can be used to call the operations from Python scripts. The wrapper is located in `pysimwild/simwild/simwild.py`.

The main function demonstrates how to use the wrapper functions. The input data can be found in the [data repository](https://github.com/wildmeshing/data2/tree/main/models).

The wrapper is lightweight and works exactly as the command line interface. The only difference is that instead of providing a JSON file, one can call the wrapper functions directly with the required parameters.
The input and output is still stored on the disk.

#### Python Wrapper Installation

Install the wildmeshing package first, as described in the [README](../../README.md).

To install the SimWild wrapper, execute:

```bash
python -m pip install -e /components/simwild/wmtk/components/simwild/pysimwild
```

For a full documentation of the wrapper functions call `help(simwild)` in Python after importing the module.

## Operations

All operations produce a .msh file as output. This file can be used as input for the next operation. The only exceptions are the entry functions `lines_to_tri` and `surf_to_tet`, which produce a .msh file as output, but do not require a .msh file as input.

#### 1. Mesh Generation from Surfaces

The entry point is the tet/tri mesh generation from surface/line meshes.

<details>
<summary>Python Example</summary>

```python
from simwild import simwild

meshes = ["circle.obj","ellipse.obj","rectangle.obj"]
tags = ["circle", "ellipse", "rectangle"]
# for 2D input use:
simwild.lines_to_tri(meshes=meshes, tags=tags, output="m1")
# for 3D input use:
simwild.surf_to_tet(meshes=meshes, tags=tags, output="m1")
```

</details>

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_double_sphere_notop_3d.json)

#### 2. Remeshing

Due to other operations, the mesh quality may deteriorate. In this case, one can perform remeshing to improve the mesh quality.

<details>
<summary>Python Example</summary>

```python
mesh = "m5"
sizing_field = [
    {"tags": "overlap", "length_rel": 3e-2},  # finer in overlap region
    {"tags": "_", "length_rel": 2e-1}  # coarser in ambient region
]
simwild.remeshing(mesh=mesh, output="m7", sizing_field=sizing_field, num_threads=10)
```

</details>

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_remeshing_2d.json)

#### 3. Replace Tags

Any input surface is assigned a volumetric tag. These tags may fulfill different purposes. With the `"replace_tags"` operation, one can replace tags (or intersections of tags) with new ones.

<details>
<summary>Python Example</summary>

```python
mesh = "m1"
tags_in = ["blob1", "blob2", "ellipse & rectangle"]
tags_out = ["blob", "blob", "_"]
simwild.replace_tags(mesh=mesh, tags_in=tags_in, tags_out=tags_out, output="m2")
```

</details>

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_replace_tags_3d.json)

#### 4. Resolve Intersections

If two regions overlap, one can use `resolve_intersections`. The overlapping regions are removed and split into the two regions.

<details>
<summary>Python Example</summary>

```python
mesh = "m2"
tags = [["rectangle", "circle"], ["rectangle", "blob"]]
simwild.resolve_intersections(mesh=mesh, tags=tags, output="m3")
```

</details>

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_resolve_intersections_3d.json)

#### 5. Tag Priority

If a tet contains multiple tags, only the one with the highest priority is kept.

<details>
<summary>Python Example</summary>

```python
mesh = "m4"
tags = ["overlap", "gap", "L", "R"]
simwild.tag_priority(mesh=mesh, tags=tags, output="m5")
```

</details>

#### 6. Tight Seal Topo

If there are holes between two regions, one can use `tight_seal_topo` to close the holes. The holes are split into the two regions, similar to `resolve_intersections`.

<details>
<summary>Python Example</summary>

```python
mesh = "m2"
tags = [["L", "R"]]
simwild.tight_seal_topo(mesh=mesh, tags=tags, output="m3")
```

</details>

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_tight_seal_3d.json)

#### 7. Keep Largest Connected Component

Remove all but the largest connected component of a given tag.

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/simwild_keep_lcc_2d.json)

#### 8. Topological Offsets

Create a topological offset around a given tag. Details on the algorithm can be found in the [paper](https://dl.acm.org/doi/10.1145/3731157).

<details>
<summary>Python Example</summary>

```python
mesh = "m3"
offset_selection = "L & R"  # interface between L and R
offset_output_tags = ["gap"]  # create a region called "gap"
target_distance = 0.1
simwild.topological_offset(
    mesh=mesh, offset_selection=offset_selection,
    offset_output_tags=offset_output_tags,overwrite_tags=False,
    target_distance=target_distance, output="m4"
    )
```

</details>
