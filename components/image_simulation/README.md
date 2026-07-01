# Image Simulation

This component provides multiple tools (called operations in the JSON) to generate and modify tet meshes.

## Python Wrapper

For convenience, we provide a [Python wrapper](pysimwild/simwild.py) for the image simulation component. The wrapper can be used to call the operations from Python scripts. The wrapper is located in `pysimwild/simwild.py`.

The main function demonstrates how to use the wrapper functions. The input data can be found in the [data repository](https://github.com/wildmeshing/data2/tree/main/models).

The wrapper is lightweight and works exactly as the command line interface. The only difference is that instead of providing a JSON file, one can call the wrapper functions directly with the required parameters.
The input and output is still stored on the disk.

#### Python Wrapper Installation

Install the wildmeshing package first, as described in the [README](../../README.md).

To install the SimWild wrapper, execute:

```bash
python -m pip install -e /components/image_simulation/wmtk/components/image_simulation/pysimwild
```

For a full documentation of the wrapper functions call `help(simwild)` in Python after importing the module.

## Operations

#### 1. Mesh Generation from Surfaces

The entry point is the tet mesh generation from surface meshes.

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/image_simulation_double_sphere_notop_3d.json)

#### 2. Remeshing

This component always produces a .msh file as output. This file can be used as input for the next operation.
If the result of the initial meshing is not satisfactory, one can perform further remeshing. Remeshing is the default operation and does not need to be explicitly specified in the JSON.

#### 3. Replace Tags

Any input surface is assigned a volumetric tag. These tags may fulfill different purposes. With the `"replace_tags"` operation, one can replace tags (or intersections of tags) with new ones.

[JSON Example](https://github.com/wildmeshing/data2/blob/main/integration_tests/image_simulation_replace_tags_3d.json)
