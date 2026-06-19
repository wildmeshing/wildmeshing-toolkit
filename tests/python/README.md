# Python integration tests

These tests mirror the C++ integration tests in
[`tests/integration_tests.cpp`](../integration_tests.cpp): the same list of JSON
inputs from [`data/integration_tests`](../../data/integration_tests) is driven
through the `pywmtk` bindings the same way the C++ `wmtk_wrapper` / `app/main.cpp`
drives it (parse JSON, inject `json_input_file`, dispatch on `application`).

For every input the test checks — like the C++ `CHECK_NOTHROW` — that the
binding runs to completion without raising. In addition it verifies that the
binding produces **the same result** as the native `wmtk_app`:

* **Serial inputs** (`num_threads` <= 1) are deterministic, so the output mesh
  files are compared byte-for-byte against `wmtk_app`'s output (`.log` files are
  ignored — they differ only by timing).
* **Multithreaded inputs** (`num_threads` > 1, e.g. `tetwild_octocat`) use the
  parallel scheduler, whose ordering is non-deterministic; their output is *not*
  expected to be identical, so only termination is checked.

## Running

The bindings must be installed in the active environment, and (for the result
comparison) the `wmtk_app` binary must be built. Using the venv created under
`build/`:

```bash
# build the C++ app + the bindings (see repo README / app/pywmtk/README.md)
build/venv/bin/python -m pip install pytest
build/venv/bin/python -m pytest tests/python -v
```

## Configuration (environment variables)

| Variable        | Default                        | Purpose                                   |
| --------------- | ------------------------------ | ----------------------------------------- |
| `WMTK_DATA_DIR` | `<repo>/data/integration_tests`| Directory holding the integration JSONs.  |
| `WMTK_APP`      | `<repo>/build/app/wmtk_app`    | Native CLI used as the result reference.  |

If `wmtk_app` is not found the result comparison is skipped (with a message) and
only the termination check runs.
