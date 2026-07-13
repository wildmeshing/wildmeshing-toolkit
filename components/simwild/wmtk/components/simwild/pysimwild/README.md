# simwild

Python package for the image-to-simulation pipeline: WMTK mesh operations
(surf_to_tet, lines_to_tri, remeshing, resolve_overlaps, tight_seal,
topological offset, ...) plus the PolyFEM-backed ops (minimum separation,
Laplacian interface smoothing, full simulation).

## Build & install

Three pieces, in order:

**1. The `wildmeshing` bindings** (required — simwild delegates all tag-
expression parsing and WMTK ops to them). From the wildmeshing-toolkit root:

```bash
pip install .            # builds the C++ bindings (takes a while)
```

Development alternative — reuse an existing CMake build instead of a wheel:

```bash
cmake -S . -B build -DWMTK_PYBIND=ON
cmake --build build -j --target wildmeshing
export PYTHONPATH=/path/to/wildmeshing-toolkit/build/bin
```

**2. This package** (pure Python). From this directory:

```bash
pip install -e .         # editable; or `pip install .`
```

**3. PolyFEM** (only for the polyfem ops: minimum_separation,
laplacian_smoothing, polyfem_sim). Build `PolyFEM_bin` and point the ops
at it:

```bash
cmake -S /path/to/polyfem -B /path/to/polyfem/build -DCMAKE_BUILD_TYPE=Release
cmake --build /path/to/polyfem/build -j --target PolyFEM_bin
export POLYFEM_BIN=/path/to/polyfem/build/PolyFEM_bin
```

`POLYFEM_BIN` is the **only** way the polyfem ops locate the binary — they
raise if it is unset. (If the link fails with `access beyond end of merged
section`, add `-DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld"` — a binutils bug
with debug-info merging.)

## Usage

```python
from simwild import simwild as wm

wm.surf_to_tet(meshes=["a.stl", "b.stl"], tags=["tag_0", "tag_1"],
               output="out/m1")
wm.resolve_overlaps(mesh="out/m1.msh", tags=[["tag_0", "tag_1"]],
                    output="out/m2")

# polyfem-backed ops share the same interface; parameters are validated
# against the spec.json packaged with each op
wm.minimum_separation(
    mesh="out/m2.msh",
    collision_pairs=[[{"region": "tag_0", "filter": "ambient"},
                      {"region": "tag_1", "filter": "ambient"}]],
    sep=1.5e-3,
    output="out/m3")
wm.laplacian_smoothing(mesh="out/m3.msh", interfaces=["tag_0"],
                       output="out/m4")
```

Selections everywhere are *the boundary of `region`, kept where the outside
cell satisfies `filter`* — both are Boolean tag expressions
(`&`, `|`, `!`, parentheses); a bare string is a region (whole boundary).

## Layout

```
simwild/
  simwild.py            # all user-facing ops (WMTK + polyfem)
  polyfem_ops/
    mesh_core.py        # tagged-mesh loading, expressions, face selection
    constraints.py      # interface selection -> polyfem constraint artifacts
    polyfem_utils.py    # shared polyfem machinery
    spec.py             # validator for the per-op spec.json files
    minimum_separation/ # runner (__init__.py) + spec.json
    laplacian_smoothing/
    polyfem_sim/        # full simulation (+ msh_boundary_extractor)
```

## Tests

```bash
python -m pytest tests
```

Tier-1 runs anywhere (the bindings are found in `<toolkit>/build/bin`
automatically if not installed). The end-to-end tests (measured minimum
separation, smoothing roughness decrease) run iff `POLYFEM_BIN` is exported.
