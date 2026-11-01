# The life of a component call

A component is a pair of C++ functions in `wmtk::components::<name>`. `wmtk_app`, the Python
module, the integration tests and C++ programs all call one of the two. This page follows a call
from the command line to the component's mesh and back, with
[shortest edge collapse](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/shortest_edge_collapse/wmtk/components/shortest_edge_collapse/shortest_edge_collapse.cpp)
as the example.

## The two entry points

Every component declares both, in `wmtk/components/<name>/<name>.hpp`:

```cpp
namespace wmtk::components::shortest_edge_collapse {

void shortest_edge_collapse(nlohmann::json json_params);
ComponentResult shortest_edge_collapse(nlohmann::json json_params, const MeshMap& meshes);

}
```

The first is the **file route**: inputs and outputs are files named in the options. The second is
the **memory route**: inputs are entries of `meshes`, nothing is written, and the outputs come
back as a `ComponentResult`, a pair of named output meshes and a JSON report
([`src/wmtk/MeshData.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/MeshData.hpp)).
Both build an `io::ComponentIO` and call the same `run()`:

```cpp
void shortest_edge_collapse(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult shortest_edge_collapse(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return context.take_result();
}
```

## How the callers find the components

`wmtk_app` and the bindings name no component in their code. `add_component(<name>)`
([`components/cmake/add_component.cmake`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/cmake/add_component.cmake))
appends one line per component to six headers in `app/`, which
[`components/CMakeLists.txt`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/CMakeLists.txt)
empties at the start of every configure:

| Header | The line it gets | Included by |
|---|---|---|
| `components_include.hpp` | `#include <wmtk/components/<name>/<name>.hpp>` | `wmtk_app`, the bindings, the integration tests |
| `components_map.hpp` | `components_map["<name>"] = ` the file overload | `wmtk_app`, the bindings, the integration tests |
| `components_memory_map.hpp` | `memory_components["<name>"] = ` the memory overload | the bindings |
| `components_spec_include.hpp` | `#include <<name>_spec.hpp>` | `wmtk_app`, the bindings |
| `components_spec_map.hpp` | `components_spec_map["<name>"] = ` the spec's accessor | `wmtk_app`, the bindings |
| `components_python.hpp` | `WMTK_PYTHON_BINDINGS(<name>)`, only if `python/src/<name>_bindings.cpp` exists | the bindings |

The maps are filled where the headers are included, inside a function:

```cpp
std::map<std::string, std::function<void(nlohmann::json)>> components_map;
// include auto-generated map
#include "components_map.hpp"
```

The headers are generated: edit `add_component`, not them, and reconfigure after adding a
component. `add_component` also links the component into `wmtk_app`, into the Python module
(with `WMTK_PYBIND`) and into `wmtk_integration_tests` (with `WMTK_BUILD_INTEGRATION_TESTS`).

## `wmtk_app`

[`app/main.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/app/main.cpp) takes
three options: `-j <file>`, `--list` and `--options <name>`. With `-j` it

1. parses the JSON file, and requires an `application` key;
2. removes `$schema` (an editor's, see [JSON Schemas](../json-schemas.md)), and sets `input_dir`
   to the directory of the JSON file, so that the component resolves the paths in the file
   against it;
3. looks the application up in `components_map` (`Application ... unknown` otherwise);
4. warns about every top-level key the component's spec does not have (see
   [below](#unknown-options));
5. calls the file overload with the JSON.

`main` does not catch what the component throws: a component fails with
`log_and_throw_error`, which logs the message, and the process ends on the uncaught exception.

## Inside the component

`run()` takes the same steps in every component.

**1. The route.** `component_io.prepare(json_params, "<name>")` does nothing on the file route.
On the memory route it sets `application`, empties `input_dir`, `output`, `log_file` and
`report`, and, for simwild, tetwild and triwild, wraps a single `input` string into a list.

**2. Validation and defaults.** The component checks the JSON against its spec and fills in the
defaults:

```cpp
const auto spec =
    jse::embed::wmtk_shortest_edge_collapse_spec::shortest_edge_collapse_spec::spec();
jse::JSE spec_engine;
bool r = spec_engine.verify_json(json_params, spec);
if (!r) {
    log_and_throw_error(spec_engine.log2str());
}
json_params = spec_engine.inject_defaults(json_params, spec);
```

The spec is compiled in: `add_component` runs `jse_add_spec` on `<name>_spec.json`, which
generates `<name>_spec.hpp`, declaring the accessor above, and a source file holding the JSON,
both in `generated/<name>/` in the build directory. The [json-spec-engine](https://github.com/geometryprocessing/json-spec-engine)
checks types, required keys and allowed values; with `strict` off, its default, it accepts keys
no rule covers, and the component then never reads them. Only topological_offset sets
`spec_engine.strict = true` and refuses them. tetwild, triwild and simwild do this step through
`utils::verify_and_setup_logger`
([`src/wmtk/utils/DriverPrologue.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/DriverPrologue.hpp)),
which also opens `log_file`.

**3. The inputs.** `component_io.path(root, json_params["input"])` is the file path, resolved
against `input_dir` by `utils::resolve_path` (against the current directory when `input_dir` is
empty), on the file route, and the mesh's name on the memory route. `component_io.triangles(...)`
then reads the file, or takes the named `MeshData` and checks its attributes, and cleans both the
same way: duplicate vertices (within `remove_duplicate_eps`), degenerate faces and unreferenced
vertices go.

**4. The mesh.** The component builds its own mesh class from the arrays:

```cpp
ShortestEdgeCollapse m(v, num_threads, !use_sample_envelope);
wmtk::set_preallocation_factor_from_json(m, json_params);
m.set_use_link_condition(use_link_condition);
m.create_mesh(v.size(), tri, modified_v, envelope_size, boundary_envelope_size);
```

**5. The operation loop.** Here `m.collapse_shortest(target_verts)`, which runs one
`ExecutePass` over every edge ([writing an operation](writing-an-operation.md)). tetwild, triwild,
simwild and topological_offset run `mesh_improvement()` ([the Wild driver](wild-driver.md)).

**6. The output.** `consolidate_mesh()` compacts the mesh; the file route writes `output` (when
it is not empty: the memory route emptied it); the memory route fills `component_io.result`:

```cpp
if (component_io.memory())
    component_io.result.first["mesh"] =
        io::triangle_data(m, [&](size_t v) { return m.vertex_attrs[v].pos; });
component_io.result.second = {
    {"vertices", m.get_vertices().size()},
    {"triangles", m.get_faces().size()}};
```

## The memory route

`io::ComponentIO`
([`src/wmtk/io/ComponentIO.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/io/ComponentIO.hpp))
is the only place that knows which route a call is on (`memory()`); the rest of the component
asks it for inputs and hands it outputs. On the memory route:

- A selector (`"input": "surface"`) names an entry of the `MeshMap`. A missing one throws
  `Unknown input mesh`; every one is `validate()`d; an attribute the component does not accept
  throws `Unknown mesh attribute`.
- Nothing is written. A recorded history goes into the report as bytes (`save_history`, then
  `take_result()` puts it under `"history"`, and a volume history under `"volume_history"`).
- The component logs to a console logger of its own, not to a file logger the caller set up.

On both routes, `ComponentIO` holds a process-wide mutex for the duration of the call: component
calls in one process run one at a time.

The contract of each component's inputs, the `MeshData` layout and the NumPy dictionaries are
specified in [component I/O](../in-memory-io.md).

## The Python bindings

The module `wildmeshing._core` is built from
[`python/src/pywildmeshing.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/python/src/pywildmeshing.cpp)
and `mesh_conversion.cpp`; `python/wildmeshing/__init__.py` re-exports it. For every component
in `memory_components`, the module defines one function with two overloads:

- `wm.<name>(options, meshes)`: the memory route. `call_component` (`mesh_conversion.cpp`)
  converts the meshes the selectors name (the keys `input`, `input_edges`, `input_points`,
  `surface_mesh`, `uv_mesh`, `envelope`, `tet_mesh`, `output_tet_mesh` and `features`), releases
  the GIL, calls the memory overload and returns `(outputs, report)` as NumPy dictionaries.
- `wm.<name>(options)`: the file route, as `wmtk_app` runs it. It adds `application` if it is
  missing and releases the GIL. It does not set `input_dir`, so relative paths resolve against
  the current directory.

The docstring of each function is built from its spec, as `wmtk_app --options` prints it:
`help(wm.qslim)` lists every option. Next to the components:

- `wm.components()`: the names of the components, from the spec map.
- `wm.spec(name)`: a component's spec, the rules of its `*_spec.json`, as a Python list.
- `wm.wildmeshing(options)` and `wm.wildmeshing(options, meshes)`: dispatch on
  `options["application"]`, for code written against the older single entry point.

A component with Python classes or functions of its own puts them in
`python/src/<name>_bindings.cpp`, defining `wmtk::python::bind_<name>(module)`; `add_component`
compiles the file into the module and lists it in `components_python.hpp`, and the module calls it.
simwild and tracking have one.

## The spec map

[`app/component_specs.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/app/component_specs.hpp)
reads the specs for `wmtk_app` and the bindings: the root rule's `doc` (the rule with pointer
`/`), the top-level options in the spec's order, each option's type, default and the first
sentence of its `doc`.

`wmtk_app --list` prints every component with the first sentence of its root `doc`.
`wmtk_app --options <name>` prints the root `doc`, the URL of the component's reference page, and
one line per option:

```text
$ wmtk_app --options shortest_edge_collapse
shortest_edge_collapse

Simplifies a triangle mesh by collapsing its shortest edges first (Hoppe 1996, Progressive Meshes), until the vertex count reaches the target. With a positive eps_rel, every collapse keeps the surface inside an envelope around the input.

Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/shortest_edge_collapse/):
  application (string, required): Application name must be shortest_edge_collapse.
  input (string, required): Triangular input mesh.
  output (string, default "out.obj"): Output file name (with extension).
  ...
```

So a spec's root `doc` should open with a sentence that says what the component does, and each
option's `doc` with a sentence that stands alone. The full text of each `doc` goes into the
[reference](../reference/index.md) ([writing documentation](documentation.md)).

### Unknown options

Since most components ignore a key their spec does not have, a misspelled option would silently
keep its default. `component_specs::unknown_options` lists the top-level keys that are not
options of the spec (`input_dir` and `$schema` excepted), and the callers warn about each:

- `wmtk_app`: ``tetwild has no option `eps_rell` (wmtk_app --options tetwild lists them)``.
- The bindings, on both overloads of `wm.<name>` and of `wm.wildmeshing`:
  ``tetwild has no option `eps_rell` (see help(wildmeshing.tetwild))``.

The warning does not stop the run, and a direct C++ call does not warn. topological_offset, with
its strict validation, then refuses the key.
