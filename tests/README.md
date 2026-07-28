# Integration tests

There are two layers of integration testing, both driven by the same list of
per-test config JSONs in the **data2** repository
(`data/integration_tests/integration_tests.json`):

1. **Termination test** (`integration_tests.cpp`, C++/Catch2 target
   `wmtk_integration_tests`) — runs every listed test through the components and
   checks it does not throw. It does *not* look at the output.
2. **Golden-hash test** (`integration_hashes.py`, ctest `integration_hashes`) —
   runs each *blessed* test through the `wmtk_app` CLI single-threaded, hashes
   every output file, and fails if a hash does not match the reference stored in
   `integration_tests.json`. This makes the outputs **deterministic and
   reproducible across compilers, OSes and CPU architectures**.

This document is about layer 2: how it works and how to (re)generate the hashes.

## TL;DR — regenerate the hashes

```bash
# 1. Build wmtk_app with reproducible floating point.
cmake -S . -B build -DWMTK_FP_STRICT=ON
cmake --build build --target wmtk_app -j

# 2. (Re)bless one or more tests: run them and store their output hashes.
python3 tests/integration_hashes.py generate --only qslim_octocat.json,triwild_puzzle.json
#   ...or re-bless every already-blessed test:
python3 tests/integration_hashes.py generate
#   ...or bless everything:
python3 tests/integration_hashes.py generate --all

# 3. Verify (this is what CI / ctest runs):
python3 tests/integration_hashes.py check          # blessed tests only
ctest --test-dir build -R integration_hashes -V

# 4. Commit the updated integration_tests.json in the data2 repo (see below).
```

`generate` **writes** the hashes into `data/integration_tests/integration_tests.json`;
`check` only **reads** them and exits non-zero on any mismatch (wrong hash, missing
output, or extra output). `list` prints which tests are blessed.

## Why the hashes are reproducible

Bit-identical output across platforms requires two things, both provided here:

* **Single-threaded execution.** The driver forces `num_threads = 1` for every
  test, removing the non-deterministic ordering of the parallel scheduler.
* **`-DWMTK_FP_STRICT=ON`.** This compiles the *entire* build (toolkit,
  components, app, and third-party code such as Eigen, libigl, geogram and
  polysolve) with:
  * no FMA contraction and no fast-math (`-ffp-contract=off -fno-fast-math`,
    MSVC `/fp:strict`) — the main source of cross-*compiler* divergence; and
  * `EIGEN_DONT_VECTORIZE` — forces Eigen's scalar code path so SIMD reductions
    sum in the same order on every architecture. Without it, ARM (NEON) and x86
    (SSE/AVX) differ at ~1 ULP, which can cascade into a completely different
    mesh.

  Because `EIGEN_DONT_VECTORIZE` also changes Eigen's alignment/ABI, it is set
  **globally** (in the top-level `CMakeLists.txt`, before any target is
  configured) so every translation unit agrees. **Always regenerate the hashes
  from a build configured with `-DWMTK_FP_STRICT=ON`, and prefer a clean build**
  so third-party libraries are rebuilt with the flag too — an incremental
  reconfigure updates the compile flags but may not recompile already-built
  dependency objects (force it with `cmake --build build --clean-first` if
  needed).

Hashes have been verified byte-identical between **macOS-arm64** and
**Linux-x86_64**. `.log` files are ignored (they contain timestamps).

## Schema

Each entry in the `integration_tests` list is either:

* a plain string — the test is run (for termination) but **not** hash-checked
  (not yet blessed), e.g.

  ```json
  "simwild_remeshing_2d.json"
  ```

* an object with the config filename and the golden hashes of its outputs — the
  test is **hash-checked**, e.g.

  ```json
  {
      "file": "qslim_octocat.json",
      "hashes": { "out.obj": "89cfd53e…c69c11f4" }
  }
  ```

This lets tests be added to the golden set **one at a time**: bless a test only
once its output is confirmed reproducible on every target platform. The per-test
config JSONs themselves are never modified (they are schema-validated by the
components), so the hashes live only in `integration_tests.json`.

## Adding a test to the golden set

1. Build with `-DWMTK_FP_STRICT=ON` (clean build recommended).
2. `python3 tests/integration_hashes.py generate --only <config>.json`
3. Verify it is reproducible on every target platform (at minimum one Linux-x86
   and one macOS-arm machine): copy the updated `integration_tests.json` to the
   other machine, build there with `-DWMTK_FP_STRICT=ON`, and run
   `python3 tests/integration_hashes.py check --only <config>.json`.
   * If the hashes match everywhere, keep it blessed.
   * If they differ, the test still has non-determinism (often a
     platform-dependent tie-break or a use of a not-yet-scalar dependency). Leave
     it unblessed (revert its entry to a plain string) until fixed.

## The hashes live in the data2 repo

`data/` is a separate repository (`github.com/wildmeshing/data2`), pinned by
`cmake/wmtk_data.cmake` (`GIT_TAG`). The build checks out exactly that commit, so:

* Commit the updated `integration_tests.json` to **data2** and push it.
* Bump `GIT_TAG` in `cmake/wmtk_data.cmake` to the new data2 commit, and commit
  that to the toolkit. Only then does a fresh build (and CI) see the new hashes.

During local iteration the build's `ExternalProject` step re-checks-out the
pinned commit whenever the data stamp is cleaned (e.g. `--clean-first`), which
reverts uncommitted edits to `integration_tests.json`. Commit your hashes to a
data2 branch so they are easy to restore
(`git -C data checkout <your-branch> -- integration_tests/integration_tests.json`).

## Options

```
python3 tests/integration_hashes.py --help
  --app PATH            wmtk_app to use (default build/app/wmtk_app or $WMTK_APP)
  --threads N           threads to force (default 1)

  generate [--only a.json,b.json | --all]
  check    [--only …] [--include-unblessed]   # non-zero exit on mismatch
  list
```

`WMTK_DATA_DIR` overrides the test-data directory (default
`data/integration_tests`).
