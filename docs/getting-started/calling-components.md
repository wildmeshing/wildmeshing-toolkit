# Calling components

Every component has two entry points, with the same options:

- **On files**: inputs and outputs are files, named in the options, as on the command line.
- **In memory**: inputs are meshes passed by name, outputs come back as meshes with a report, and
  nothing is written (no mesh, report, log or debug files).

On the same input, both give the same mesh; only the order of the cells in a `.msh` file can
differ (it groups cells by region). The full specification is in the
[component I/O design notes](../in-memory-io.md).

## The two routes

=== "Python"

    ```python
    --8<-- "docs/examples/python/calling_components.py:routes"
    ```

    `wm.<component>(options)` works on files and returns nothing;
    `wm.<component>(options, meshes)` works in memory and returns `(outputs, report)`.
    `outputs["mesh"]` is the result; some components return more meshes (TetWild also returns
    `surface`, its tracked surface, and `envelope`). The component releases the Python GIL while
    it runs; calls run one at a time per process.

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/calling_components.cpp:routes"
    ```

    Each component has both overloads:

    ```cpp
    void name(nlohmann::json options);
    ComponentResult name(nlohmann::json options, const MeshMap& meshes);
    ```

    `MeshMap` is `std::map<std::string, MeshData>`; `ComponentResult` is the pair of output
    meshes and the JSON report.

=== "Command line"

    `wmtk_app -j options.json` runs the file route. `application` names the component; input
    paths are relative to the JSON file, and outputs are written to the working directory.

In memory, the options name inputs instead of files (`"input": "sphere"`) and need no
`application`, `output` or `input_dir`.

## Mesh data

A mesh is vertex positions and blocks of cells by type, each with optional named fields:

=== "Python"

    ```python
    --8<-- "docs/examples/python/calling_components.py:dict"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/calling_components.cpp:meshdata"
    ```

- `V`: positions, one row per vertex and three columns (a 2D mesh has z = 0). Float64.
- `cells`: blocks keyed by type, `point`, `edge`, `triangle`, `quad`, `tetrahedron`, `prism` or
  `pyramid`, each with a `connectivity` array (Int64, one row per cell) and optional
  `attributes`. Prism and pyramid corners are in VTK order.
- `vertex_attributes` and a block's `attributes`: named fields,
  `{"values": array, "labels": [...]}`, Float64, Int64 or UInt8, one row per vertex or cell.

Regions are a UInt8 cell field `tags`, one column per region, labelled by its name. A cell can be
in several regions:

```python
mesh["cells"]["tetrahedron"]["attributes"] = {
    "tags": {"values": np.array([[1, 1], [1, 0]], dtype=np.uint8),
             "labels": ["region_a", "region_b"]},
}
```

See [Concepts: regions](../concepts/regions.md) for how components select regions by
expressions of their names.

The bindings copy what they read into C++ arrays, so inputs may be read-only or strided; outputs
own their arrays and can go straight into the next call.

## Files

=== "Python"

    ```python
    --8<-- "docs/examples/python/calling_components.py:files"
    ```

`read_mesh` reads `.msh` (Gmsh 4.1), `.obj` and libigl's surface formats (`.stl`, `.off`, `.ply`,
...); `write_mesh` writes `.msh` and `.vtu`. `wm.save(outputs, stem)` writes every output mesh of a
result, and `wm.load(stem)` reads back exactly what it wrote, so a pipeline resumed from a
checkpoint continues as it would have.

## Reports

The report is a JSON object (a dictionary in Python): counts, timings, quality statistics, what
the component logged. On the file route, the `report` option names a file to write it to. Each
component's report is described with its tutorial.

## Histories

With `history_output`, a component records every operation it commits. On files, the history
goes to the file `history_output` names; in memory, any non-empty value records it and the
report returns it as bytes:

```python
--8<-- "docs/examples/python/calling_components.py:history"
```

From a history, [tracking](../tutorials/tracking.md) maps points and fields between the input and
the output.
