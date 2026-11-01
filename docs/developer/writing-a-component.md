# Writing a component

A component is a directory under `components/`, a CMake call that registers it, and a spec that
documents its options. Registered, it is a `wmtk_app` application, a Python function and a C++
library (`wmtk::<name>`) at once. This page is the checklist, in the order the work goes; the
existing components are the examples, shortest edge collapse the smallest.

## 1. The directory

```text
components/<name>/wmtk/components/<name>/
    CMakeLists.txt          add_component, the sources, the tests
    <name>_spec.json        the options
    <name>.hpp, <name>.cpp  the two entry points
    <Mesh>.h, <Mesh>.cpp    the mesh class, its hooks and its passes
    tests/
        CMakeLists.txt      add_component_test, the test sources
        test_<name>.cpp
```

The three `<name>` levels are what the include path needs: `add_component` adds
`components/<name>` to the include directories, so the header is
`#include <wmtk/components/<name>/<name>.hpp>`, as the generated headers write it.

## 2. CMake

The component's `CMakeLists.txt`, after
[shortest edge collapse's](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/shortest_edge_collapse/wmtk/components/shortest_edge_collapse/CMakeLists.txt):

```cmake
set(COMPONENT_NAME shortest_edge_collapse)
add_component(${COMPONENT_NAME})

if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    ShortestEdgeCollapse.cpp
    ShortestEdgeCollapse.h
    shortest_edge_collapse.hpp
    shortest_edge_collapse.cpp
)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC
    wmtk::toolkit
)

if(${WMTK_COMPONENT_TESTS})
    add_subdirectory(tests)
endif()
```

and one line in [`components/CMakeLists.txt`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/CMakeLists.txt):

```cmake
add_subdirectory("<name>/wmtk/components/<name>")
```

`add_component(<name>)`
([`components/cmake/add_component.cmake`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/cmake/add_component.cmake))
defines the option `WMTK_ENABLE_COMPONENT_<name>` (on), the library `wmtk_<name>` and its alias
`wmtk::<name>`, embeds the spec (`jse_add_spec`), appends the component to the generated headers
in `app/`, and links it into `wmtk_app`, the Python module and the integration tests
([the life of a component call](component-call.md#how-the-callers-find-the-components)).
Reconfigure after adding it: the headers are written at configure time.

A component that uses another links it, as tetwild and simwild link `wmtk::shortest_edge_collapse`
to simplify their input.

## 3. The entry points

In `<name>.hpp`, in `namespace wmtk::components::<name>`, exactly these two signatures, which the
generated maps take the address of:

```cpp
void <name>(nlohmann::json json_params);
ComponentResult <name>(nlohmann::json json_params, const MeshMap& meshes);
```

Write one `run(nlohmann::json, io::ComponentIO&)` and call it from both, as every component does
([the life of a component call](component-call.md#the-two-entry-points)). In `run`:

- start with `component_io.prepare(json_params, "<name>")`;
- validate and inject the defaults with the embedded spec,
  `jse::embed::wmtk_<name>_spec::<name>_spec::spec()` (or `utils::verify_and_setup_logger`, which
  also opens `log_file`);
- get every input through `component_io` (`path`, `paths`, `triangles`, `edges`, `mesh`), so the
  memory route reads the `MeshMap` and the file route the files;
- on the memory route, put the output meshes in `component_io.result.first` and write no file;
  put what a caller may want to check (counts, energies, times) in the report,
  `component_io.result.second`;
- throw with `log_and_throw_error` when the result is not what was asked for, at least when
  `throw_on_fail` is set: the integration tests only see exceptions.

The memory route's contract, input by input, is in [component I/O](../in-memory-io.md); add the
component's row to its table. If a mesh selector has a new name, add it to the list of keys
`call_component` converts in
[`python/src/mesh_conversion.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/python/src/mesh_conversion.cpp),
or the Python memory route will not pass that mesh.

## 4. The spec

`<name>_spec.json` is a list of [json-spec-engine](https://github.com/geometryprocessing/json-spec-engine)
rules. It is what validates the options, and it is their documentation: `wmtk_app --options`,
`help(wildmeshing.<name>)`, the [reference](../reference/index.md) and the
[JSON Schemas](../json-schemas.md) are all generated from it. The root rule names the required
and optional keys and says what the component does:

```json
{
  "doc": "Simplifies a triangle mesh by collapsing its shortest edges first (Hoppe 1996, Progressive Meshes), until the vertex count reaches the target. With a positive eps_rel, every collapse keeps the surface inside an envelope around the input.",
  "pointer": "/",
  "type": "object",
  "required": ["application", "input"],
  "optional": ["output", "input_dir", "num_threads", "target_rel", "eps_rel", "..."]
}
```

and every option has a rule of its own, with a `doc`:

```json
{
  "pointer": "/target_rel",
  "type": "float",
  "default": 0.1,
  "doc": "Target number of vertices relative to input number."
}
```

- The first sentence of the root `doc` is the component's line in `wmtk_app --list`; the first
  sentence of an option's `doc` is its line in `--options`. Make both stand alone.
- Give every optional key a `default`; `inject_defaults` fills it in, and `run` can then read
  `json_params["..."]` without checking.
- Every spec has `application`, with `"options": ["<name>"]`. Give it `input_dir` too, which
  `wmtk_app` sets, and, where they apply, the keys most components share: `output`,
  `num_threads`, `log_file`, `report`, `throw_on_fail`.
- Options for diagnostics, unfinished work or the tests start with `DEBUG_`, `EXPERIMENTAL_` or
  `TESTING_`: the reference folds them away.
- A key the spec does not have is accepted and ignored (with a warning from `wmtk_app` and the
  bindings), unless the component sets `spec_engine.strict = true`. So do not read a key that is
  not in the spec.

`python docs/tools/gen_reference.py --check` fails when the root rule or any other rule has no
`doc`, when the spec has no entry in `docs/components.yml`, or when that entry names an option
the spec does not have, a group that does not exist or a tutorial that is not there.

## 5. Unit tests

`tests/CMakeLists.txt`:

```cmake
if(NOT BUILD_TESTING)
    return()
endif()
if(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
    return()
endif()

add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_<name>.cpp
)
target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
```

`add_component_test(<name>)`
([`components/cmake/add_component_test.cmake`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/cmake/add_component_test.cmake))
builds the Catch2 executable `wmtk_test_<name>`, linked with the component, `wmtk::toolkit` and
`wmtk::data` (for `WMTK_DATA_DIR`), and registers it with ctest as one entry of the same name. A
test executable that is built but not registered never runs, in CI or locally; check that it is:

```bash
ctest --test-dir build -N | grep wmtk_test_<name>
```

Tag the cases too slow for a Debug build `release_only` ([testing](testing.md#slow-cases)). If
the component has a memory route, add a case to
[`tests/test_component_io.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/tests/test_component_io.cpp)
under `#ifdef WMTK_TEST_<name>`, and the component to the list in `components/CMakeLists.txt`
that links `wmtk_component_io_tests` and defines that macro.

## 6. An integration config

An integration test is a whole run from a JSON file in the
[data repository](https://github.com/wildmeshing/data2): `integration_tests/<name>_<model>.json`,
with its paths relative to that directory (`"input": "../models/Octocat.obj"`). Reuse a model
from `models/` when one fits; add a small one otherwise. Set `throw_on_fail` (or the component's
equivalent) so that a bad result fails the test, and keep the run short: every Release CI job
runs every listed config.

1. Add the config, and any new model, to data2, and list the config in
   `integration_tests/integration_tests.json`; open a pull request there.
2. Once it is merged, set `GIT_TAG` in
   [`cmake/wmtk_data.cmake`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/cmake/wmtk_data.cmake)
   to the data2 commit, with a line in the comment above it saying what changed
   ([contributing](contributing.md#the-data-repository)).

## 7. The documentation

1. An entry in [`docs/components.yml`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/docs/components.yml):

    ```yaml
    shortest_edge_collapse:
      title: Shortest edge collapse
      group: surface
      tutorial: tutorials/shortest_edge_collapse.md
      key_options: [input, output, target_rel, eps_rel, boundary_eps_rel, use_link_condition, use_sample_envelope]
      papers:
        - "[Hoppe 1996, Progressive Meshes](https://hhoppe.com/pm.pdf)"
    ```

    `group` is one of the `groups` at the top of the file; the reference page of a `wip`
    component opens with a work-in-progress warning. The reference page and its place in the
    navigation are generated from this entry and the spec; `key_options` come first on it.
2. A tutorial, `docs/tutorials/<name>.md`, in the order [writing documentation](documentation.md#a-tutorial-page)
   gives, and its entry in the `nav` of `mkdocs.yml`.
3. Its examples, `docs/examples/{json,python,cpp}/<name>.*`, which the tests build and run
   ([writing documentation](documentation.md#examples)).

## 8. Python

Nothing to do: the module defines `wm.<name>` with both overloads from the generated maps, and
its docstring from the spec. Classes or functions of the component's own go in
`python/src/<name>_bindings.cpp`, defining `wmtk::python::bind_<name>(pybind11::module_&)`;
`add_component` compiles the file when it exists ([the life of a component
call](component-call.md#the-python-bindings)).

## Before the pull request

- [ ] `wmtk_app --list` shows the component, and `wmtk_app --options <name>` its options.
- [ ] `ctest --test-dir build -N` lists `wmtk_test_<name>`, and it passes in Debug and Release.
- [ ] `python docs/tools/gen_reference.py --check` reports no problems.
- [ ] `mkdocs build --strict` builds, with the new tutorial.
- [ ] The integration config runs with `wmtk_app -j`, and the data pin names the data2 commit
  that has it.
- [ ] The code is formatted ([contributing](contributing.md#formatting)).
