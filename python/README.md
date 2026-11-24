# wildmeshing (Python)

One package, `wildmeshing`:

- `wildmeshing`: the bindings. Every component is a function taking its JSON options, the same
  keys as the command line's JSON (see each component's `*_spec.json`).
- `wildmeshing.save` / `wildmeshing.load`: checkpoints of memory results.
- `wildmeshing.polyfem_ops`: the PolyFEM-backed operations.
- `wildmeshing.viewer`: a polyscope viewer for a chain of components, each stage's outputs by
  label (volume, face and edge tags) with their envelopes; `python -m wildmeshing.viewer
  <checkpoint stem> ...` views saved stages. Needs polyscope (`pip install -e ".[viewer]"`).
- `wildmeshing.pipeline`: a pipeline of components in one JSON file, run in memory, each stage
  cached under a key and saved only where asked (see [Pipelines](#pipelines)).
- `wildmeshing.gui`: a window to build, run and view a pipeline: `python -m
  wildmeshing.gui [<pipeline.json>]`. Needs polyscope.

The bindings are the compiled module `wildmeshing._core`, built from `python/src`; the
pure-Python part of the package is `python/wildmeshing`.

Installation, the two ways to call a component, mesh dictionaries, regions and every
component's options are in the documentation:
[installation](../docs/getting-started/installation.md#the-python-package),
[calling components](../docs/getting-started/calling-components.md),
[Python API](../docs/python-api.md), [regions](../docs/concepts/regions.md); the mesh
dictionaries and both routes in full: [docs/in-memory-io.md](../docs/in-memory-io.md).

## Envelopes

simwild returns `outputs["envelope"]`, the surface the mesh must stay close to, when it has
one: the inputs it held the regions to, labelled by region, as tetwild and triwild return theirs.
Pass it on explicitly (`m1` a simwild result); the next run holds the mesh to it at its own eps,
so a chain stays within one eps of the original inputs. Without one, simwild derives an envelope
from the regions.

```python
m2, _ = wm.simwild({"operation": "remeshing", "input": ["m"], "envelope": "e"},
                   {"m": m1["mesh"], "e": m1["envelope"]})
```

Operations that change regions do not read the envelope given; they return a new one, the
interfaces of their result.

## Pipelines

A pipeline file names its inputs and lists its stages; an option value may refer to an input, or
to an earlier stage's output mesh, report value or option:

```json
{"inputs": {"bunny": "data/models/bunny.obj"},
 "stages": [
   {"name": "remesh", "component": "isotropic_remeshing",
    "options": {"input": {"from": "bunny", "output": "mesh"}, "eps_rel": 0.02,
                "length_rel": 0.045, "history_output": "history"}},
   {"name": "map", "component": "tracking",
    "options": {"operation": "info", "history": {"from": "remesh", "report": "history"}}}]}
```

```python
from wildmeshing.pipeline import Pipeline
results = Pipeline.load("bunny.json").run()  # {name: {"outputs", "report", "key", ...}}
```

Each stage has a key: the sha256 of its component, its options, the keys of what it reads and the
installed compiled module. A stage whose key is unchanged comes from memory or from the cache
(`<pipeline>_cache/`, checkpoints with their records) instead of running; a stage with `"save":
"<stem>"` also writes its checkpoint there. A file is used only for the key it was written for and
while it hashes as written. Each stage's C++ log is kept next to its checkpoint.

`python -m wildmeshing.gui bunny.json` edits the same file in a window: inputs, stages and
their options as each component's spec types them, a run in the background with the log, and a
view of every stage (tags, envelopes, attributes, correspondences).

A `prismatic_mesh` stage selects its regions with two tag expressions, `input_tags` (the volume) and
`offset_tags` (the bands). Until the C++ component reads these options, the pipeline rebuilds its
input: the regions `tag_0` and `offset`, and vertex labels from the offsets' correspondence.
Original curve edges and sheet triangles are retained for rod classification. Set
`"rod_mode": true` to include rod frame construction, and `"keep_background_mesh": true`
to export the constrained background. The GUI reads these options from the compiled
component's spec, so rebuild/reinstall the extension when updating the C++ implementation.
`"relative_tet_volume_floor": 0.25` optionally limits collapse and smoothing moves to
retain a quarter of every modified surviving tet's current volume, including background
(default 0). Deleted collapse cells are excluded.

Stacked correspondences such as `new_offset → old_offset → original_input` are handled
conservatively as protected junction fibers. The original links are recorded in the hybrid
report; regular optimization uses their input-root correspondence, while shared curve/sheet
vertices, input-volume vertices and affected junction stars stay fixed. This supports a
valid mixed mesh through such interfaces, but does not independently prism-mesh every
overlapping offset layer: extended overlaps can retain substantial tet regions.

## PolyFEM operations

`minimum_separation` and `laplacian_smoothing` take their options (`spec.json` next to each) as a
component does; `polyfem_sim` takes its own configuration.

- In this process, on mesh dictionaries, writing no files: `minimum_separation.separate(options,
  meshes)`, `laplacian_smoothing.smooth(options, meshes)` and `polyfem_sim.simulate(cfg, mesh)`.
  They need a wildmeshing built with `WMTK_WITH_POLYFEM`.
- On files, a `.msh` in and a `.msh` out (write a memory result with `wm.save` first):
  `minimum_separation(options)`, `laplacian_smoothing(options)` and `polyfem_sim.run(cfg)`.
  `minimum_separation` solves in this process here too, so it needs `WMTK_WITH_POLYFEM`: each of
  its solves starts from the last one it accepted, which PolyFEM_bin has no input for.

`laplacian_smoothing(options)` and `polyfem_sim.run` run PolyFEM_bin, found only through
`POLYFEM_BIN`; they raise if it is unset:

```bash
export POLYFEM_BIN=/path/to/polyfem/build/PolyFEM_bin
```

(If the PolyFEM link fails with `access beyond end of merged section`, add
`-DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld"`: a binutils bug with debug-info merging.)

Selections are _the boundary of `region`, kept where the outside cell satisfies `filter`_, both
region expressions; a bare string is a region's whole boundary.

- `minimum_separation`: deform regions apart until their distance reaches `sep`. `sep` is a hard
  floor; `rtol` bounds the overshoot.
- `laplacian_smoothing`: fair material interfaces with one PolyFEM solve (AMIPS, fitting,
  Laplacian; no contact).
- `polyfem_sim`: a PolyFEM simulation on an extracted simulation mesh, materials and boundary
  conditions selected by region name; see `polyfem_sim.msh_boundary_extractor`.

## Type stubs

`wildmeshing/_core.pyi` describes the compiled module for editors and type checkers, with the
docstrings the components' specs give (`help(wm.qslim)`); `py.typed` tells them to read it. The
build writes it from the module it has just built (`python/src/CMakeLists.txt`), so it always
matches: every `pip install`, and a CMake build whose Python has `pybind11-stubgen` 3 and numpy.
Not in git.

## Tests

The Python tests are in `tests/python`, listed in
[docs/developer/testing.md](../docs/developer/testing.md#python-tests). The pipeline's, the GUI's
and the viewer's, from the repository root:

```bash
python -m pytest tests/python/test_pipeline.py tests/python/test_gui.py tests/python/test_viewer.py
```

The tests find the bindings in `build/bin` when the package is not installed. The PolyFEM
operations' tests are in `tests/python/polyfem`, collected only with the polyfem extras installed;
the end-to-end ones are skipped while those ops raise.
