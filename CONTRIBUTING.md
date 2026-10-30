# Contributing to the Wildmeshing Toolkit

The developer guide on the documentation site describes the code, the tests and the workflow:
<https://wildmeshing.github.io/wildmeshing-toolkit/developer/>. Its
[contributing page](https://wildmeshing.github.io/wildmeshing-toolkit/developer/contributing/) has
the details of what follows.

Pull requests go to `main`. Before opening one:

- **Format the code.** C and C++ files are formatted with clang-format 21.1.8 through pre-commit
  (`.pre-commit-config.yaml`, rules in `.clang-format`). Install the hook with
  `pip install pre-commit && pre-commit install`, or run `pre-commit run --all-files`. CI fails
  on any file the hook would change.
- **Add tests, and check that they are registered.** A test that is built but not registered
  with ctest never runs. After adding one, `ctest --test-dir build -N` must list it. Component
  tests go through `add_component_test` (`components/cmake/add_component_test.cmake`), library
  tests into `TEST_SOURCES` in `tests/CMakeLists.txt`, integration configs into the
  [data repository](https://github.com/wildmeshing/data2), with the pin in
  `cmake/wmtk_data.cmake` moved to the commit that has them.
- **Document every option in the spec.** A component's options are the rules of its
  `<name>_spec.json`, and their `doc` is their only documentation: the reference pages,
  `wmtk_app --options` and the Python docstrings are generated from it.
  `python docs/tools/gen_reference.py --check` fails on a rule without a `doc`.
- **Update the documentation.** A change in what a component does updates its tutorial in
  `docs/`; check the site with `mkdocs build --strict`
  ([writing documentation](https://wildmeshing.github.io/wildmeshing-toolkit/developer/documentation/)).

The required CI checks are the Debug and Release builds and tests on Linux, macOS and Windows,
and `pre-commit`; the branch must be up to date with `main` to merge.
