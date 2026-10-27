# wildmeshing (Python)

One package, `wildmeshing`:

- `wildmeshing`: the bindings. Every component is a function taking its JSON options, the same
  keys as the command line's JSON (see each component's `*_spec.json`).
- `wildmeshing.save` / `wildmeshing.load`: checkpoints of memory results.
- `wildmeshing.polyfem_ops`: the PolyFEM-backed operations.

The mesh dictionaries and the two calling routes are specified in
[docs/in-memory-io.md](../docs/in-memory-io.md).

## Installation

From the repository root:

```bash
python -m pip install -e .              # the bindings
python -m pip install -e ".[polyfem]"   # plus gmsh, scipy, h5py for polyfem_ops
```

Development alternative, reusing a CMake build. The build tree mirrors the package in
`build/bin/wildmeshing`:

```bash
cmake -S . -B build -DWMTK_PYBIND=ON
cmake --build build -j --target wildmeshing
export PYTHONPATH=/path/to/wildmeshing-toolkit/build/bin
```

## Calling a component

- `wm.<component>(options)`: the file route. Inputs and outputs are files, as on the command
  line; `"output": "m1"` writes `m1.msh`.
- `wm.<component>(options, meshes)`: the memory route. `meshes` maps names to mesh
  dictionaries, inputs are given by name, nothing is written, and the call returns
  `(outputs, report)`. `outputs["mesh"]` is the result.

simwild takes a list of inputs, the other components a single one.

```python
import wildmeshing as wm

# file route
wm.simwild({"input": ["a.obj", "b.obj"], "input_names": ["a", "b"], "output": "m1"})

# memory route
m1, _ = wm.simwild({"input": ["a", "b"], "input_names": ["a", "b"]},
                   {"a": wm.read_mesh("a.obj"), "b": wm.read_mesh("b.obj")})
m2, _ = wm.simwild({"operation": "resolve_overlaps", "input": ["m"],
                    "resolve_overlaps_tags": [["a", "b"]]},
                   {"m": m1["mesh"]})
```

On the same input both routes give the same mesh. Only the cell order can differ: a `.msh`
stores cells grouped by region.

### Regions

A tagged mesh keeps its regions in the top-dimensional cells' `tags` attribute: one UInt8 column
per region, labelled by its name. A cell may belong to several regions. Components label the
background `ambient`. Options select regions by expression: `"a & b"` (overlap), `"a | b"`,
`"!a"`, parentheses.

### Envelopes

simwild returns `outputs["envelope"]`, the surface the mesh must stay close to, when it has
one. Pass it on explicitly; without one, simwild derives an envelope from the regions.

```python
m2, _ = wm.simwild({"operation": "remeshing", "input": ["m"], "envelope": "e"},
                   {"m": m1["mesh"], "e": m1["envelope"]})
```

Operations that change regions return no envelope.

## Files and checkpoints

- `wm.read_mesh(path)` reads `.msh`, `.obj` and libigl's surface formats (`.stl`, `.off`, ...);
  `wm.write_mesh(path, mesh)` writes `.msh` and `.vtu`. A `.msh` holds one mesh and nothing
  else.
- `wm.save(outputs, stem)` writes `stem.msh`, plus `stem_envelope.msh` when the outputs have an
  envelope. `wm.load(stem)` returns exactly the arrays `save` wrote, in the same order, so a
  pipeline resumed from a checkpoint continues as it would have without one.

```python
wm.save(m2, "out/m2")
m2 = wm.load("out/m2")
```

## Operations

Full options in each component's spec.

| Call | Does | Main options |
|---|---|---|
| `wm.simwild` on surfaces (`.obj`) | Tet mesh of closed surfaces, one region each | `input_names`, `eps_rel` |
| `wm.triwild` on polylines | Triangle mesh of closed curves, one region each | `input_names`, `eps_rel` |
| `simwild`, `"operation": "remeshing"` | Improve quality | `eps_rel`, `stop_energy` |
| `simwild`, `"replace_tags"` | Replace regions (or overlaps) by others | `replace_tags_in`, `replace_tags_out` |
| `simwild`, `"resolve_overlaps"` | Split each overlap between its two regions | `resolve_overlaps_tags` |
| `simwild`, `"tag_priority"` | Keep only the highest-priority region per cell | `tag_priority` |
| `simwild`, `"tight_seal_topo"` | Close gaps between regions | `tight_seal_tag_sets` |
| `simwild`, `"keep_lcc"` | Keep the largest connected component | `keep_lcc_tags` |
| `simwild`, `"fill_holes_topo"` | Fill a region's holes | `fill_holes_tags` |
| `wm.topological_offset` | New region at a distance around a selection ([paper](https://dl.acm.org/doi/10.1145/3731157)) | `offset_selection`, `offset_output_tags`, `target_distance` |
| `wm.manifold_extraction` | Make a selection manifold | `tag_selection`, `fill_tags` |

```python
m3, _ = wm.simwild({"operation": "replace_tags", "input": ["m"],
                    "replace_tags_in": ["a & b"], "replace_tags_out": ["b"]},
                   {"m": m2["mesh"]})
m4, _ = wm.topological_offset({"input": "m", "offset_selection": "a",
                               "offset_output_tags": ["gap"], "target_distance": 0.05},
                              {"m": m3["mesh"]})
```

## PolyFEM operations

`minimum_separation`, `laplacian_smoothing` and `polyfem_sim` (full simulation) run on files:
a `.msh` in, a `.msh` out. Write a memory result with `wm.save` first.

`minimum_separation` and `laplacian_smoothing` raise for now: their options (`spec.json`) are not
wired to the engine yet. Planned: `minimum_separation(options)`, validated against its spec like
a component.

They find PolyFEM only through `POLYFEM_BIN`, and raise if it is unset:

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

## Tests

From the repository root:

```bash
python -m pytest python/tests
```

The tests find the bindings in `build/bin` when the package is not installed. The PolyFEM
end-to-end tests are skipped while those ops raise.
