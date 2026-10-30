# Python API

The `wildmeshing` package: the compiled bindings (`wildmeshing._core`, built from `python/src`)
and a pure-Python part (`python/wildmeshing`). Every function and class has a docstring:
`help(wildmeshing.tetwild)` lists the component's options. [Installation](getting-started/installation.md#the-python-package)
explains how to build it.

```python
import wildmeshing as wm
```

## Components

One function per component, with two calls:

`wm.<component>(options)`
:   On files, as `wmtk_app` runs the same options. Returns `None`.

`wm.<component>(options, meshes) -> (outputs, report)`
:   In memory. `meshes` maps names to mesh dictionaries, and the options name their inputs;
    `outputs` maps names to mesh dictionaries (`outputs["mesh"]` is the result) and `report` is a
    dictionary. Nothing is written.

| Function | Tutorial | Reference |
|---|---|---|
| `wm.shortest_edge_collapse` | [tutorial](tutorials/shortest_edge_collapse.md) | [options](reference/shortest_edge_collapse.md) |
| `wm.qslim` | [tutorial](tutorials/qslim.md) | [options](reference/qslim.md) |
| `wm.isotropic_remeshing` | [tutorial](tutorials/isotropic_remeshing.md) | [options](reference/isotropic_remeshing.md) |
| `wm.tetwild` | [tutorial](tutorials/tetwild.md) | [options](reference/tetwild.md) |
| `wm.triwild` | [tutorial](tutorials/triwild.md) | [options](reference/triwild.md) |
| `wm.simwild` | — | [options](reference/simwild.md) |
| `wm.topological_offset` | [tutorial](tutorials/topological_offset.md) | [options](reference/topological_offset.md) |
| `wm.manifold_extraction` | [tutorial](tutorials/manifold_extraction.md) | [options](reference/manifold_extraction.md) |
| `wm.tracking` | [tutorial](tutorials/tracking.md) | [options](reference/tracking.md) |
| `wm.prismatic_mesh` | work in progress | [options](reference/prismatic_mesh.md) |
| `wm.c1_simplification` | work in progress | [options](reference/c1_simplification.md) |

`wm.wildmeshing(options[, meshes])`
:   The same, with the component named by `options["application"]`, as in a `wmtk_app` file.

`wm.components() -> list[str]`
:   The components' names.

`wm.spec(name) -> list[dict]`
:   A component's spec: the rules of its options (pointer, type, default, doc, ...), from which
    its [reference](reference/index.md) is generated.

An option a component does not have logs a warning, and has no effect (except in
`topological_offset`, which refuses it).

## Meshes and files

Mesh dictionaries are described in [Calling components](getting-started/calling-components.md#mesh-data).

`wm.read_mesh(path) -> dict`
:   Reads `.msh` (Gmsh 4.1), `.obj`, and libigl's surface formats (`.stl`, `.off`, `.ply`, ...).

`wm.write_mesh(path, mesh)`
:   Writes `.msh` or `.vtu`.

`wm.save(outputs, stem)`, `wm.load(stem) -> dict`
:   A checkpoint of a memory result: `<stem>.msh`, and `<stem>_envelope.msh` when the result has
    an envelope. `load` returns exactly the arrays `save` wrote, in the same order, so a pipeline
    resumed from a checkpoint continues as it would have without one.

`wm.read_history(path) -> bytes`, `wm.write_history(path, history)`
:   A history file (`history_output`, `volume_history_output`) as the bytes a memory call returns
    in its report, and back.

## Maps

Built from the history an application recorded; see the [tracking tutorial](tutorials/tracking.md).

`wm.Tracking(history)`, `wm.Tracking.load(path)`
:   The map between a surface application's input and its output.
    `to_output(ids, bary)` and `to_input(ids, bary)` map points given as faces (int64, `(n,)`)
    and barycentric coordinates (float64, `(n, 3)`), and return them on the other side, `-1` for
    a point with no image. `input_faces`, `input_triangles` and `output_triangles` name the faces
    of each side; `build_warnings` counts where the map is not a bijection.

`wm.CurveTracking(history)`, `wm.CurveTracking.load(path)`
:   The same for curves (TriWild): points on segments, with `feature_points`.

`wm.VolumeTracking(...)`, `wm.VolumeTracking.load(...)`
:   The map of a tet mesh's volume, region by region, from the surface and volume histories that
    TetWild and SimWild record with `volume_history_output`.

## Regions

`wm.Expression(expr, known_names)`
:   A parsed tag expression (names, `&`, `|`, `!`, parentheses, `_`; see
    [Concepts: regions](concepts/regions.md)). `eval(tags)` evaluates it on a set of tag names;
    `has_unknown_names()` is true when it names a tag outside `known_names`.

## PolyFEM operations

`wildmeshing.polyfem_ops` runs PolyFEM-based operations on `.msh` files (install with
`pip install ".[polyfem]"`, and set `POLYFEM_BIN`): see
[python/README.md](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/python/README.md#polyfem-operations).
