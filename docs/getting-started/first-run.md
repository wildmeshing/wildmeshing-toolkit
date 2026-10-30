# A first run

Isotropic remeshing of the Stanford bunny (35,947 vertices, 69,451 faces) to a uniform edge
length of 2% of its bounding box diagonal, three ways. Run each from the repository root, after
[building](installation.md) with tests enabled (the meshes are in `data/models`).

=== "Command line"

    `wmtk_app` reads a JSON file of options. `application` names the component; input paths are
    relative to the JSON file, and outputs are written to the working directory:

    ```json
    --8<-- "docs/examples/json/first_run.json"
    ```

    ```bash
    mkdir -p first_run
    cp data/models/bunny.obj docs/examples/json/first_run.json first_run/
    cd first_run && ../build/app/wmtk_app -j first_run.json
    ```

    It writes `first_run/bunny_remeshed.obj`.

=== "Python"

    The same options, as a dictionary. Without meshes, the component works on files:

    ```python
    --8<-- "docs/examples/python/first_run.py:file"
    ```

    With meshes, it works in memory: the inputs are named, the result comes back as NumPy arrays,
    and nothing is written:

    ```python
    --8<-- "docs/examples/python/first_run.py:memory"
    ```

=== "C++"

    The memory interface, on `wmtk::MeshData`:

    ```cpp
    --8<-- "docs/examples/cpp/first_run.cpp:run"
    ```

    Link `wmtk::isotropic_remeshing`; see [Installation](installation.md#using-the-library-from-your-own-cmake-project).

All three give the same mesh: 2,034 vertices and 3,851 faces. In memory, the report holds what the
command line logs: the edge lengths before and after, the valences, and the operations refused
because they would fold the surface (136 here).

The options not given take their defaults. Every component lists its options:

```bash
./build/app/wmtk_app --options isotropic_remeshing
```

```python
help(wm.isotropic_remeshing)
```

and the [reference](../reference/isotropic_remeshing.md) documents each one. A misspelled option
is not an error: the component ignores it and logs a warning
(`isotropic_remeshing has no option ...`). Editors can check option files before a run with the
[JSON Schemas](../json-schemas.md).

## Next

- [Calling components](calling-components.md): the two routes, mesh data and reports.
- [Isotropic remeshing](../tutorials/isotropic_remeshing.md): what the options do.
