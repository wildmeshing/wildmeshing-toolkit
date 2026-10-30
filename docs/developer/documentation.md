# Writing documentation

The site is MkDocs with the Material theme, built from `docs/` by `mkdocs.yml`. Everything on it
is either written in `docs/` or generated from the code on every build; nothing generated is
committed.

## Building the site

```bash
pip install -r docs/requirements.txt
mkdocs serve            # http://127.0.0.1:8000, rebuilt on every change
mkdocs build --strict   # as CI does: a broken link or anchor fails the build
```

CI (`.github/workflows/docs.yml`) builds the site with `--strict` on every pull request and
deploys it from `main`, with the Doxygen C++ reference under `/api/`.

## What is generated

`docs/tools/gen_reference.py` reads every `components/<name>/wmtk/components/<name>/<name>_spec.json`
and `docs/components.yml`, and writes, at build time:

- `reference/<name>.md`: every option of the component, with its type, default and doc. The
  options in `key_options` (components.yml) come first; `DEBUG_`, `EXPERIMENTAL_` and `TESTING_`
  options are folded away. Each option has an anchor: link to one as
  `../reference/tetwild.md#eps_rel`.
- `reference/<name>_defaults.json`: the options that have a default, with it.
- `schemas/<name>.json` and `schemas/wmtk_app.json`: JSON Schemas for editors (see
  [JSON Schemas](../json-schemas.md)).
- The reference's navigation: a new component appears on its own.

So **the spec is the documentation of an option**: improve an option's description by editing its
`doc` in the spec, and `help(wildmeshing.<name>)`, `wmtk_app --options <name>` and the reference
change with it. `gen_reference.py --check` (run in CI) fails when a rule has no `doc`, when a spec
has no entry in `components.yml`, or when an entry names an option its spec does not have.

## Examples

Every example on the site is a file under `docs/examples/`, included into pages, built and run
by the tests. A page never contains example code of its own that is not in one of these files.

| Directory | What | How it is tested |
|---|---|---|
| `docs/examples/json/<name>.json` | a `wmtk_app` input file | copied into a directory with the meshes it names, run with `wmtk_app -j` |
| `docs/examples/python/<name>.py` | a script | run with `python <name>.py` in a directory of its own |
| `docs/examples/cpp/<name>.cpp` | a program | built as `wmtk_doc_example_<name>`, linking every component, and run |

All three are registered in ctest when CMake runs with `-DWMTK_BUILD_DOC_EXAMPLES=ON` (and
`BUILD_TESTING`), in Release builds only: `doc_example_cpp_<name>` for each program, and
`doc_examples_python_json` for the scripts and JSON files together
(`tests/python/test_docs_examples.py`).

Conventions, so that the files run the same from the repository root, in the tests and on a
reader's machine:

- **Input meshes** come from `data/models` (the [data repository](https://github.com/wildmeshing/data2),
  cloned by CMake when tests are enabled). Python and C++ read the directory from the environment
  variable `WMTK_DATA`, defaulting to `data/models`, so they run from the repository root as they
  are:

    ```python
    DATA = Path(os.environ.get("WMTK_DATA", "data/models"))
    ```

    ```cpp
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";
    ```

    A JSON file names its meshes by file name alone (`"input": "bunny.obj"`): `wmtk_app` resolves
    input paths against the JSON file's directory (and writes outputs to the working directory),
    and the test copies the meshes there and runs in it.
- **Outputs** go to the working directory, under names that say what they are
  (`bunny_remeshed.msh`).
- **Small and fast**: an example takes seconds, a minute at most in Release. Pick a small mesh
  and options that keep the run short; a tutorial explains what to change for quality.
- **Sections**: mark the part a page shows with snippet markers, and include it by name:

    ```python
    # --8<-- [start:memory]
    outputs, report = wm.qslim(...)
    # --8<-- [end:memory]
    ```

    ```markdown
    --8<-- "docs/examples/python/qslim.py:memory"
    ```

    A section name may not start with a digit (`[start:2d]` fails the build; `[start:two_d]`
    works). Inside a fenced block on the page, with the language set (```` ```python ````). Paths are
    relative to the repository root. Including a whole file (`--8<-- "docs/examples/json/qslim.json"`)
    is right for JSON.
- An example with a test of its own starts with `# docs-example: tested by <test file>`, and
  `test_docs_examples.py` skips it (the tracking tutorial's pipeline is one:
  `tests/python/test_tracking_correspondences.py`).

Run them while writing:

```bash
cmake -B build -DWMTK_BUILD_DOC_EXAMPLES=ON ...
cmake --build build --target wmtk_doc_example_qslim wmtk_app wildmeshing
ctest --test-dir build -R 'doc_example' --output-on-failure
```

## A tutorial page

One page per component, `docs/tutorials/<name>.md`, in this order:

1. **What it does**, in a paragraph, and when to use it (and when another component fits better).
2. **A first run**, in three tabs over the same run, with the same options:

    ````markdown
    === "Command line"

        ```json
        --8<-- "docs/examples/json/qslim.json"
        ```

        ```bash
        wmtk_app -j qslim.json
        ```

    === "Python"

        ```python
        --8<-- "docs/examples/python/qslim.py:memory"
        ```

    === "C++"

        ```cpp
        --8<-- "docs/examples/cpp/qslim.cpp:run"
        ```
    ````

    The tabs are linked across the site (`content.tabs.link`): a reader who picks Python sees
    Python everywhere.
3. **Key settings**: the options in the component's `key_options`, each with what it changes and
   when to change it, a link to its reference entry, and the numbers a reader needs (what the
   default does on the example, what a coarser or finer value gives). Measure them: run the
   example and say what came out.
4. **Outputs**: the files written, the fields on the mesh, what the report holds.
5. **Recipes and pitfalls**: what goes wrong in practice and what to do.
6. **See also**: the reference page, related tutorials, the paper.

Write in plain declarative sentences, present tense, without marketing. Numbers come from runs,
not from memory: if a page says "about 1,000 faces", an example produced it.
