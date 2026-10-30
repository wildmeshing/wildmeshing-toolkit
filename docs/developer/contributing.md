# Contributing

Changes reach `main` through pull requests on
[GitHub](https://github.com/wildmeshing/wildmeshing-toolkit). This page covers what a pull request
needs to pass, and the two places a change can reach outside the code: the documentation and the
data repository. [`CONTRIBUTING.md`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/CONTRIBUTING.md)
at the root of the repository is the short version.

## Formatting

C and C++ code is formatted with clang-format, by the rules in
[`.clang-format`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/.clang-format)
(based on Google's style, 4-space indents, 100 columns).
[`.pre-commit-config.yaml`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/.pre-commit-config.yaml)
pins the version, clang-format 21.1.8 from `mirrors-clang-format`, and applies it to C, C++ and
CUDA files. Install the hook once, and every commit formats the files it changes:

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files      # the whole tree, as CI checks it
```

[`.github/workflows/pre-commit.yml`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/.github/workflows/pre-commit.yml)
runs the same hook on every pull request and every push to `main`, and fails on any file it would
change. Another clang-format version can format the same code differently: format with the
pinned one, through pre-commit, rather than with the one an editor bundles.

Python, CMake and JSON files have no formatter in the hooks; follow the file around the change.

## Pull requests

- Work on a branch of your own, by convention `<user>/<topic>` (`uday/offset-carry-correspondence`).
- Open the pull request against `main`. Say what changes and why; for a change that moves
  results, give the before and after on the models it affects (element counts, energies, times),
  and keep the description current as commits are added.
- `main` is protected. A pull request can be merged when these checks pass: `Linux (Debug)`,
  `Linux (Release)`, `macos-latest (Debug)`, `macos-latest (Release)`,
  `Windows 2025 (VS 2026) (Debug)`, `Windows 2025 (VS 2026) (Release)` (the jobs of
  `continuous.yml`, see [testing](testing.md#in-ci)) and `pre-commit`. The Linux arm64 jobs run
  as well, and are not required.
- The branch must be up to date with `main` before it merges: merge `main` into it when another
  pull request lands first, and let the checks run again.
- Pull requests are merged with a merge commit, and the branch is deleted.

Before opening one:

- [ ] The code is formatted (`pre-commit run --all-files`).
- [ ] New tests are registered: `ctest --test-dir build -N` lists them ([testing](testing.md)).
- [ ] The tests pass in Debug and in Release; a change to a component also runs its integration
  configs (`-DWMTK_BUILD_INTEGRATION_TESTS=ON`).
- [ ] A new or changed option is documented in its spec, and
  `python docs/tools/gen_reference.py --check` passes.
- [ ] The pages that describe the change are updated, and `mkdocs build --strict` passes.

## Documentation

The site is built from `docs/` and from the components' specs; [writing documentation](documentation.md)
describes how. On every pull request,
[`.github/workflows/docs.yml`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/.github/workflows/docs.yml)
runs `gen_reference.py --check` and `mkdocs build --strict`, and uploads the built site as the
artifact `docs-preview`; on `main` it deploys it. What a change to the code asks of the site:

- **An option** is documented by its `doc` in the spec, and nowhere else: the reference,
  `wmtk_app --options` and the Python docstrings are generated from it. Changing an option's
  meaning or default means changing its `doc`.
- **A behaviour a tutorial describes** (a default, a number a tutorial measured, an output field)
  means updating the tutorial, and rerunning its examples when the numbers come from them.
- **An example** under `docs/examples` is built and run with `-DWMTK_BUILD_DOC_EXAMPLES=ON`,
  which the Release jobs of `continuous.yml` set; run them locally first
  (`ctest --test-dir build -R doc_example`), in a Release build.
- **A new component** gets an entry in `docs/components.yml` and a tutorial
  ([writing a component](writing-a-component.md#7-the-documentation)).

## The data repository

The meshes and integration configs live in [data2](https://github.com/wildmeshing/data2). The
build clones it into `data/` (or the directory in the `WMTK_DATA_ROOT` cache variable) and checks
out the commit pinned in [`cmake/wmtk_data.cmake`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/cmake/wmtk_data.cmake):

```cmake
GIT_REPOSITORY https://github.com/wildmeshing/data2.git
# ...
GIT_TAG 89911f53375f21eb6e8463caaca5bf07ce9c22c7
```

`data/` is a clone checked out at that commit, a detached `HEAD`. To change the data:

1. Make a branch in data2, from `data/` or a clone of your own, and add the models and configs.
   A config is listed in `integration_tests/integration_tests.json` to run in CI; one in a
   group of its own (`challenging_low_stop_energy_models.json`,
   `topological_offset_models.json`, `manifold_extraction_models.json`) runs only when its
   hidden test case is asked for ([testing](testing.md#integration-tests)).
2. Open a pull request on data2 and merge it.
3. In the toolkit's pull request that needs the data, set `GIT_TAG` to the data2 commit, and add
   a line to the log in the comment above it: the commit, the date, the data2 pull request or
   branch, and what changed for which config. The log is the history of the pin: read it before
   moving it.

A config that changes in data2 changes what CI runs only when the pin moves, so a pull request
that needs new data, or new values in a config, carries the pin bump. Reconfigure and build to
check the new pin out in `data/`.

Keep models small: every CI job of every pull request clones the whole repository, and every
Release job runs the listed configs.
