# Testing

The suites in the table are registered with ctest; the few that are not are run by hand, as the
sections below say. A test that is built but not registered never runs, in CI or locally: after
adding one, check that `ctest --test-dir build -N` lists it.

| Suite | Source | ctest entries | Registered when |
|---|---|---|---|
| Library unit tests | the files of `TEST_SOURCES` in `tests/` | one per test case | `BUILD_TESTING` (on by default) |
| Component unit tests | `components/<name>/.../tests/` | `wmtk_test_<name>`, one per component | `BUILD_TESTING` and `WMTK_COMPONENT_TESTS` (both on by default) |
| Memory route | `tests/test_component_io.cpp` | one per test case | as the component tests |
| Integration tests | `tests/integration_tests.cpp`, configs in `data/integration_tests` | `Integration_Tests` | `WMTK_BUILD_INTEGRATION_TESTS=ON` |
| Python | `tests/python/` | `python_memory_io`, `python_tracking`, `python_tracking_correspondences` | the Python module is built and its interpreter has NumPy; the last in non-Debug builds only |
| Documentation examples | `docs/examples/` | `doc_example_cpp_<name>`, `doc_examples_python_json` | `WMTK_BUILD_DOC_EXAMPLES=ON`, non-Debug builds only |

## Library unit tests

`wmtk_tests` is one Catch2 executable built from the files listed in `TEST_SOURCES` in
[`tests/CMakeLists.txt`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/tests/CMakeLists.txt):
a new test file goes into that list. `catch_discover_tests` registers every test case as a ctest
entry of its own, under its name. The tests link `wmtk::data`, so they read the data repository
through `WMTK_DATA_DIR`:

```cpp
const std::string root(WMTK_DATA_DIR);
```

Small meshes built in code are in
[`src/wmtk/utils/examples/`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/src/wmtk/utils/examples)
(`TriMesh_examples.hpp`: `single_triangle()`, `two_triangles()`, ...).

`-DWMTK_DEBUG_BRUTE_FORCE_OPS=ON` compiles a check into `TriMesh`'s split and collapse that
compares each result with one rebuilt from the whole mesh. It makes every operation linear in
the mesh size: for tests on small meshes only.

## Component unit tests

Each component's `tests/CMakeLists.txt` calls `add_component_test(<name>)`
([`components/cmake/add_component_test.cmake`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/cmake/add_component_test.cmake)),
which builds `wmtk_test_<name>` and registers it as **one** ctest entry:

```cmake
add_test(NAME ${COMPONENT_TEST_NAME} COMMAND ${COMPONENT_TEST_NAME} --allow-running-no-tests --durations yes)
```

`--durations yes` prints the time of every case, which is how a case too slow for the Debug jobs
shows up in the CI log. To run one case, run the executable with its name or a tag:

```bash
ctest --test-dir build -R wmtk_test_qslim --output-on-failure
build/components/qslim/wmtk/components/qslim/tests/wmtk_test_qslim "[boundary]"
```

`wmtk_component_io_tests`
([`tests/test_component_io.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/tests/test_component_io.cpp),
built from `components/CMakeLists.txt`) tests the memory route: components called on named
meshes, and the memory and file routes giving the same result. Each component's cases are
compiled only when the component is (`#ifdef WMTK_TEST_<name>`).

## Integration tests

With `-DWMTK_BUILD_INTEGRATION_TESTS=ON`, `wmtk_integration_tests`
([`tests/integration_tests.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/tests/integration_tests.cpp))
runs every config listed in `data/integration_tests/integration_tests.json`, the way `wmtk_app`
does (the JSON, `input_dir` set to its directory, the component from the generated map):

```cpp
for (const auto& input_file : input_files) {
    const path& f = integration_tests_dir / input_file;
    CHECK(fs::exists(f));
    CHECK_NOTHROW(wmtk_wrapper(f));
}
```

That is the whole check: a config passes if it runs without throwing. No output is compared
with a stored one. A config therefore asks its component to throw when the result is wrong:
`throw_on_fail` (the output misses what the component promises: the target vertex count, the
stop energy), `max_expected_iterations` in tetwild
and triwild (more optimization iterations than expected), `throw_on_nonconvergence` in
topological_offset. All the configs run in one ctest entry, `Integration_Tests`, with a timeout of
7200 s.

The other cases in the file are hidden (`[.]`): Catch2 does not run them unless asked, and
`catch_discover_tests` does not register them. Each reads its own list from
`data/integration_tests`:

```bash
build/tests/wmtk_integration_tests "[challenging]"   # challenging_low_stop_energy_models.json: minutes to hours each
build/tests/wmtk_integration_tests "[offset]"        # topological_offset_models.json
build/tests/wmtk_integration_tests "[manifold]"      # manifold_extraction_models.json
```

A new config goes into the data repository and its list, and the data pin moves to the data2
commit that has it ([writing a component](writing-a-component.md#6-an-integration-config)).

## Python tests

The tests in [`tests/python`](https://github.com/wildmeshing/wildmeshing-toolkit/tree/main/tests/python)
run the bindings. Three are registered in `tests/CMakeLists.txt`, when the module is built
(`WMTK_PYBIND`, on by default) and the interpreter CMake found imports NumPy:

| ctest entry | File | What it runs |
|---|---|---|
| `python_memory_io` | `test_memory_io.py` | the memory route through the bindings, NumPy only |
| `python_tracking` | `test_tracking.py` | histories as bytes and files, `wm.Tracking`, `wm.CurveTracking`, `wm.VolumeTracking` |
| `python_tracking_correspondences` | `test_tracking_correspondences.py` | the tracking tutorial's pipeline (`docs/examples/python/tracking_correspondences.py`): the dense bunny remeshed, tetrahedralized, and every vertex mapped through both histories and back; its counts are ranges, since a run with threads changes slightly from one run to the next; non-Debug builds only, timeout 1800 s |

The others are run by hand or elsewhere:

- `test_docs_examples.py`: the documentation's examples, below.
- `test_smoke.py`: imports the module and its error paths; the wheel builds run it with
  `test_memory_io.py` and `test_tracking.py` on every wheel (`test-command` in
  `pyproject.toml`).
- `test_integration.py`: the integration configs through the bindings, compared byte for byte
  with `wmtk_app`'s output when the config is serial (`python -m pytest tests/python`; see its
  `README.md`).

`python/tests` holds the tests of `wildmeshing.polyfem_ops`, run with `python -m pytest
python/tests` and gmsh installed.

## Documentation examples

With `-DWMTK_BUILD_DOC_EXAMPLES=ON`, every `docs/examples/cpp/<name>.cpp` is built as
`wmtk_doc_example_<name>`, linked with every component, and in a non-Debug build registered as
`doc_example_cpp_<name>`; `doc_examples_python_json` (`tests/python/test_docs_examples.py`) runs
the Python scripts and, through `wmtk_app -j`, the JSON files. They check that each example runs
without an error. [Writing documentation](documentation.md#examples) has the conventions.

## Slow cases

Debug builds run 10 to over 100 times slower than Release. A case that takes more than a fraction
of a second in Debug is tagged `release_only`: hidden in Debug builds, run as usual in Release.
The pattern, at the top of the test file:

```cpp
// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif
```

and on each slow case:

```cpp
TEST_CASE("sec-open-surface-boundary-is-preserved", "[test_sec][boundary]" + release_only)
```

Commit [`e8494ffb4f`](https://github.com/wildmeshing/wildmeshing-toolkit/commit/e8494ffb4f)
introduced it for the 95 cases over 0.3 s in a macOS Debug build or 5 s on
Windows Debug, taking a macOS Debug ctest to 27.5 s; tetwild's and simwild's whole runs were
already hidden in Debug. A case that should not run anywhere by default, a benchmark on a large
model say, gets a hidden tag of its own (`[.slow]`), and the reason in a comment.

## Running the tests locally

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DWMTK_BUILD_INTEGRATION_TESTS=ON
cmake --build build -j
ctest --test-dir build -N                                   # what is registered
ctest --test-dir build -E Integration_Tests --output-on-failure
ctest --test-dir build -R Integration_Tests --output-on-failure
```

The first build clones the data repository into `data/`.

## In CI

[`.github/workflows/continuous.yml`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/.github/workflows/continuous.yml)
runs on every pull request and every push to `main`, in eight jobs:

| Job (check name) | Runner | Configs |
|---|---|---|
| `Linux` | `ubuntu-latest` | Debug, Release |
| `Linux arm64` | `ubuntu-24.04-arm` | Debug, Release |
| `macos-latest` | `macos-latest` | Debug, Release |
| `Windows 2025 (VS 2026)` | `windows-2025-vs2026` | Debug, Release |

A check is named after its job and config, as in `Linux (Release)`.

Each job configures with `CMAKE_BUILD_TYPE` set to its config, builds with two jobs (`make -j2`;
Ninja on Windows) behind a compiler cache (ccache; sccache on Windows), and runs
`ctest --verbose --output-on-failure`, one test at a time (`CTEST_PARALLEL_LEVEL: 1`). So:

- every job runs the library, component and memory-route tests, with the `release_only` cases
  in the Release jobs only;
- the Release jobs add `-DWMTK_BUILD_INTEGRATION_TESTS=ON` and `-DWMTK_BUILD_DOC_EXAMPLES=ON`,
  and run every listed integration config and the documentation's C++ examples;
- the Linux jobs install `python3-numpy`, so they run the registered Python tests, and in
  Release the Python and JSON examples; the other jobs run them if the interpreter CMake finds
  has NumPy.

When the tests fail on Linux or macOS, a last step re-runs the component test executables under
gdb or lldb and prints the backtrace of every thread, for a crash that leaves only a signal in
the log. A manual run of the workflow (`workflow_dispatch`) with the `debug_ssh` input opens an
SSH session on a Linux or macOS runner when a step fails.

Two other workflows run on every pull request: `pre-commit.yml` checks the formatting
([contributing](contributing.md#formatting)), and `docs.yml` checks the specs
(`gen_reference.py --check`) and builds the site with `--strict`
([writing documentation](documentation.md)).
