# Component I/O

Every registered component exposes its existing file-oriented entry point and an owning,
in-memory interface:

```cpp
void name(nlohmann::json options);
ComponentResult name(nlohmann::json options, const MeshMap& meshes);
```

`MeshMap` is `std::map<std::string, MeshData>`. `ComponentResult` is a pair of named
output meshes and a JSON report. The memory overload never resolves a selector as a filename,
never changes its input meshes, and does not emit mesh, report, log, or debug files. It does not
require `application`, `input_dir`, or output filenames. Python exposes both interfaces,
using plain dictionaries of NumPy arrays for mesh data.

```cpp
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/components/qslim/qslim.hpp>

wmtk::MeshMap meshes{{"surface", wmtk::io::read_mesh("input.obj")}};
auto [outputs, report] = wmtk::components::qslim::qslim(
    {{"input", "surface"}, {"target_rel", 0.5}}, meshes);
wmtk::io::write_mesh("simplified.vtu", outputs.at("mesh"));
```

## Python

Build and install in your active environment with `python -m pip install -e .`.
Every enabled component exposes `name(options)` for the existing file interface and
`name(options, meshes)` for the memory interface. The latter returns `(meshes, report)`.
`wildmeshing(options)` remains compatible; `wildmeshing(options, meshes)` selects the memory
component from `options["application"]`. Direct component calls do not require `application`.

```python
import numpy as np
import wildmeshing as wm

surface = {
    "V": np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]], dtype=np.float64),
    "cells": {
        "triangle": {"connectivity": np.array([[0, 1, 2]], dtype=np.int64)},
    },
}
meshes, report = wm.qslim(
    {"input": "surface", "target_rel": 1.0, "eps_rel": 0.0},
    {"surface": surface},
)
positions = meshes["mesh"]["V"]
# Explicit file I/O is separate from the memory call:
wm.write_mesh("result.msh", meshes["mesh"])
loaded = wm.read_mesh("result.msh")
```

The dictionary mirrors the C++ data structure:

- `V`: an `(n, 3)` NumPy array. Floating coordinates are promoted to Float64; integer
  coordinates must convert exactly. Returned positions always use Float64.
- `cells`: a dictionary keyed by `point`, `edge`, `triangle`, `quad`, `tetrahedron`, `prism`,
  or `pyramid`. Each block contains a two-dimensional integer `connectivity` array and an
  optional `attributes` dictionary. Returned connectivity uses Int64.
- Optional `vertex_attributes` and cell `attributes` map names to
  `{"values": array, "labels": [column_names...]}`. Labels may be omitted; a bare array is also
  accepted as an unlabeled input attribute. Attribute dtypes are Float64, Int64, or UInt8.
  One-dimensional input attributes become column matrices. Returned fields always contain
  `values` and `labels`.

For example, overlapping region memberships are represented as:

```python
mesh["cells"]["tetrahedron"]["attributes"] = {
    "tags": {
        "values": np.array([[1, 1], [1, 0]], dtype=np.uint8),
        "labels": ["region_a", "region_b"],
    },
}
```

Only selected dictionary entries are converted and validated. JSON selector arrays retain
their order. Inputs may be read-only or strided arrays: the binding copies selected data into
owning C++ arrays before computation. Outputs own their NumPy storage and can be passed directly
into subsequent memory calls. Connectivity is never truncated from floating point, and Int64
attributes never pass through Float64. Computation releases the Python GIL. Component attribute
contracts and the shared readers' and writers' supported formats are the same as in C++.

Run the Python memory tests with `python -m unittest discover -s tests/python -p test_memory_io.py`,
and those of tracking -- histories and `wm.Tracking` / `wm.CurveTracking` -- with
`-p test_tracking.py`; they are also included in the wheel tests.

## MeshData

- `V` is an owning row-major Float64 matrix with three columns. Pad 2D coordinates with zero.
- `cells[CellType]` holds a row-major Int64 connectivity matrix and named attributes.
  Cell types are point (1 corner), edge (2), triangle (3), quad (4), tetrahedron (4), prism (6),
  and pyramid (5). Prism/pyramid ordering follows VTK.
- `vertex_attributes` and each cell block's `attributes` map names to `MeshAttribute`.
  `values` is a variant of row-major Float64, Int64, or UInt8 matrices. One row corresponds
  to a vertex or a row in that cell block's connectivity. Vectors are one-column matrices.
- `MeshAttribute::labels` optionally names columns. `tags` uses a binary UInt8 matrix with
  one named column per physical/material group, allowing overlapping memberships. Group
  names are labels of this fixed field, rather than arbitrary extra attribute names.
- `validate()` checks geometry, connectivity, and attribute dimensions. Coordinates must be
  finite. Attributes may contain undefined numerical diagnostics such as NaN. A cell may repeat
  a vertex: components clean degenerate input cells exactly as their file readers do.

Only top-dimensional connectivity is needed to construct an algorithm's mesh. Explicit lower
blocks identify the edges/faces carrying attributes; their row order is independent of internal
mesh IDs. Do not use internal edge/face IDs as interchange indices. Exact positions, caches,
rollback buffers, and mesh objects are not part of MeshData.

## Input contracts

Selectors retain their component-specific JSON fields. A single selector is a dictionary key;
ordered selector arrays preserve input order and existing `input_names` / transform semantics.
Unselected dictionary entries are not validated. Missing keys and incompatible inputs throw
`std::runtime_error` before computation. Unknown attributes are rejected; this interface does
not transfer arbitrary user attributes through topology changes. Geometry-only inputs ("no
attributes" below) ignore the fields `io::read_mesh` attaches to plain geometry files: vertex
`vid`, and triangle `uv` and `normal`. A component may accept, and ignore, a field of its own
results as well, so that a result can be the next call's input as it is: isotropic_remeshing's
`vertex_id`, whose fixed vertices come from `features` alone. isotropic_remeshing also takes the
triangle fields its `feature_fields` names, as on the `.msh` route: the per-face ones it keeps as
labels through the remeshing and returns on `mesh`, each in the type it was given in, with its
labels (an Int64 one within 2^53 in magnitude, which Float64 holds exactly); `uv` and `normal`,
per corner (6 and 9 columns), it neither carries nor returns: their seams are curves, in `edges`.
So `mesh` goes back in with the same per-face fields named again.

| Component | Mesh selectors | Required data |
| --- | --- | --- |
| `qslim` | `input` | Triangle geometry, no attributes |
| `shortest_edge_collapse` | `input` | Triangle geometry, no attributes |
| `isotropic_remeshing` | `input`; optional `features` | Triangle geometry, no attributes but its own `vertex_id`, ignored, and the triangle fields `feature_fields` names; `features` on the rows of `input`: edges with Int64 `edge_id`, points with Int64 `vertex_id`, or Int64 vertex `vertex_id` (-1 for none) |
| `tetwild` | `input`; `input_edges`, `input_points` | Triangle surfaces; optional curve/point geometry, no attributes |
| `triwild` | `input`; `input_points` | Curve geometry, optional free points, no attributes |
| `simwild` | `input`; optional `envelope` for tagged input | Surface/curve geometry, or tagged triangle/tet mesh |
| `topological_offset` | `input`; optional `envelope` | Tagged triangle/tet mesh |
| `manifold_extraction` | `input`; optional `envelope` | Tagged triangle/tet mesh |
| `prismatic_mesh` | `input` | Tet construction mesh with `corr_input_vid` and cell region memberships |
| `c1_simplification` | `surface_mesh`, `uv_mesh` | Surface triangle `dofs` (Float64, 36 columns), vertex `is_cone` (UInt8, 1 column); UV triangle geometry |
| `tracking` | none for `info` and `features`; `transfer`: `input`, optional `tet_mesh`; `volume`: optional `tet_mesh` and `output_tet_mesh` | The history as bytes (option `history`, not a mesh), and for `volume` the volume history as bytes (option `volume_history`). `transfer`: the triangle surface the history starts from, whose triangle attributes it carries (`uv`, `normal` per corner, any other per face), no vertex attributes; the tet mesh the history ends at. `volume`: the tet meshes the volume history starts and ends at, with their `tags` if they have them |

For tagged inputs, `tags` on the top-dimensional cells names their regions, such as `tag_0`. A
column labelled `ambient` is the background, as is a cell without a tag; a mesh without a `tags`
field is all background. Tagged results label their background `ambient` explicitly; the shared
readers and writers infer nothing from an untagged row. Existing exported diagnostics have fixed allowed
names in the adapters and may be supplied again. Extra unrelated fields are errors. As on the
`.msh` route, a lower-dimensional block of the envelope's type (triangles in a tet mesh, edges in
a triangle mesh) is the envelope, e.g. simwild's `EnvelopeSurface`; it cannot be combined with an
`envelope` input. Other lower-dimensional blocks are rejected.

The prismatic adapter accepts scalar `tag_0` and `offset_tag`, or the corresponding columns of
`tags`. `vid` defaults to vertex row IDs; absent `labels` are derived from input-volume cells and
correspondence. `corr_input_vid` names a source `vid`, with -1 for no correspondence. Existing
construction diagnostics are accepted. Enable `save_offset_correspondence` and disable offset
optimization to obtain construction correspondence for the offset-to-prismatic workflow.

C1's 36 DOF columns flatten its 12-by-3 face matrix in row order. UV vertices use `V`'s first two
columns; UV faces correspond to surface face rows. Its unfinished tetrahedral mapping route is
unchanged.

## Results and compatibility

`mesh` is the primary output key. Components also return their applicable existing secondary
geometry under fixed keys:

- `surface`: the boundary (triangles in 3D; in 2D -- triwild, 2D simwild -- the boundary edges).
  Manifold extraction returns its extracted boundary here.
- `edges`: simwild's 3D feature edges; isotropic_remeshing's feature edges, with `features` or
  `feature_fields` (its Int64 `edge_id`, and `mesh`'s Int64 `vertex_id` and per-face fields --
  in the types they were given in, with their labels -- are what its `.vtu` output holds, an
  edge block of none where there are none), which can be the next call's `features`.
- `offset`: the offset surface or front; prismatic's classified offset faces.
- `envelope`, `features`, `uv`, and manifold extraction's 3D input-complex preview `input_complex`.
- Prismatic `export_input_and_shell` adds `input_and_shell` and its classified offset faces as
  `input_and_shell_offset`; with `prism_dominant`, the report adds that export's
  `input_and_shell_hybrid` report, as in its `_hybrid.json` file.
- `history_input`: simwild's, with `history_output`, the tagged tet mesh its surface history
  starts from (the file route's `<output>_history_input.msh`); tetwild's, with
  `volume_history_output`, the tet mesh its volume history starts from (its
  `<output>_history_input.msh`).
- `history_output`: tetwild's, with `volume_history_output`, the tet mesh its volume history
  ends at, all of it before the filter (its `<output>_history_output.msh`); `mesh` is the part
  the filter keeps. For simwild the volume history ends at `mesh`.

For tetwild, triwild, simwild, topological_offset and isotropic_remeshing, `surface`, `edges` and
`offset` share the `mesh` vertex rows: their connectivity indexes the same `V`, with the same
vertex fields, so row `i` is the same vertex in every one of them, and vertices an output does not
reference are kept.
This is deliberate -- it maps a boundary vertex to its volume vertex without a search -- at the
cost of one vertex array per output. `envelope`, `features`, manifold extraction's `surface` and
prismatic's offset faces have vertices of their own.

The exporters' legacy file names map to these keys through one explicit table
(`io::default_output_keys`); an unmapped export or two exports with one key is an error, never
a silently dropped output. Reports contain the existing
algorithm counts, quality measures, and timings; prismatic reports also contain hybrid and
smoothing/remeshing reports. Report values are available without a report filename.

The existing topological-offset driver currently refuses 2D inputs; the memory overload preserves that restriction.

The file-oriented entry points retain existing configurations, relative-path conventions,
output filenames, intermediate exports, and report files. The tagged components (simwild, triwild,
topological_offset, manifold_extraction) build the same result on both routes: the file route
writes its `mesh` (the envelope stays out of the file) through `io::write_mesh`, and
reads its `.msh` input through `io::read_mesh`. An explicit per-call output policy
turns off their file observers for memory calls. Component entry points serialize their use of
the legacy process-wide loggers, temporarily using console sinks during memory calls and restoring
the previous logger afterward.

Component calls do not run concurrently. Every call -- file or memory, from C++ or Python -- holds
one process-wide lock for its whole duration, so calls from several threads run one after
another; the Python bindings release the GIL, but a second call still waits for the first. Use
`num_threads` for parallelism within a call. Existing algorithm options, filtering,
cleaning, and exact-arithmetic decisions are shared between the two routes.

## Histories

`isotropic_remeshing`, `qslim`, `shortest_edge_collapse`, `tetwild` and simwild in 3D record the
history of their surface with `history_output`, and `triwild` the history of its curves, from
which `tracking` rebuilds the map between input and output (see `docs/tracking.md`). The file
route saves it to that file. The memory route writes no file: any non-empty `history_output`
turns the recording on, and the report returns the history as `history`, the bytes of the file
the file route would write -- a `nlohmann::json` binary value in C++. On the memory route the
history starts at the meshes given and ends at the meshes returned.
`history::write_history(path, bytes)` saves the bytes as that file, unchanged, and
`history::read_history(path)` reads them back, as `io::write_mesh` and `io::read_mesh` do for a
mesh; `history::SurfaceHistory::from_bytes` and `history::CurveHistory::from_bytes` load them,
and `history::face_size(bytes)` says which they hold (3 corners a face, or 2). The history file
is its own format, not a mesh.

tetwild and simwild in 3D also record the volume around the tracked surface with
`volume_history_output` (which needs `history_output`). The memory route returns it the same
way, as `volume_history` in the report, the bytes of its file, and writes no file; with it the
meshes the volume history names its points by, as output meshes: `history_input`, where it
starts, and tetwild's `history_output`, the mesh before its filter, where it ends (simwild's
ends at `mesh`). `read_history` and `write_history` take a volume history by its header as they
take the others; `history::VolumeHistory::from_bytes` loads it and `history::is_volume_history`
says that bytes are one.

`tracking` takes the bytes as `history` on the memory route, where a string is refused, as
every memory call refuses to take a name for a file. `info` returns its report, of a surface or
of a curve history. `features`, on a curve history, returns its feature points in the report:
`feature_points`, pairs of an input vertex and the output vertex it became, -1 for none (an
Int64 array of two columns in NumPy). `transfer`, on a surface history, needs
`input`, the mesh the history starts from -- the history's own parameters name meshes of an
earlier call, or files, neither of which the memory route reads -- and returns the output
surface as `mesh`, the carried fields as triangle attributes, per face or, for `uv` and
`normal`, three values per corner (`uv`: six columns), with `input_face` (Int64), the input face
each output face comes from. With `tet_mesh`, the tet mesh the history ends at, it also returns
that mesh as `tet_mesh` with the fields on the rows of its triangle block: a tracked face already
there (a face tag) on its row, any other on a row added after them. `volume` takes the volume
history's bytes as `volume_history` -- a string is refused there too -- and, to sample points
near the tracked surface and check where they land, `tet_mesh` and `output_tet_mesh`, keys of
the meshes given (a producer's `history_input`, and its `mesh` or `history_output`); its report
holds what the file route logs (`stats`, `round_trip`, `regions_kept`, ...). `map` and
`map_volume`, whose points are text files, are refused on the memory route: points are mapped
in memory by `Tracking`, `CurveTracking` and `VolumeTracking`
(`components/tracking/.../tracking.hpp`), in Python `wm.Tracking`, `wm.CurveTracking` and
`wm.VolumeTracking`.

In Python the history is `bytes`: the bindings return a report's binary values as `bytes`, and
pass `bytes` in the options as binary values (pybind11_json alone would make them base64 text).
`wm.write_history(path, history)` and `wm.read_history(path)` are the two file functions:

```python
outputs, report = wm.isotropic_remeshing(
    {"input": "surface", "history_output": "record"}, {"surface": surface})
wm.write_history("remeshed.wmtkhist", report["history"])  # the file the file route writes
history = wm.read_history("remeshed.wmtkhist")            # the same bytes

t = wm.Tracking(history)                  # the map; wm.Tracking.load(path) from the file
ids, bary = t.to_output(faces, bary)      # int64 (n,), float64 (n, 3): points on the input
faces, bary = t.to_input(ids, bary)       # and back; -1 (and NaN) for a point with no image
```

A volume history maps the same way, with the surface history it was recorded with and the two
meshes, as a memory call returns them:

```python
outputs, report = wm.tetwild(
    {"input": "surface", "history_output": "x", "volume_history_output": "y"}, {"surface": s})
v = wm.VolumeTracking(report["history"], report["volume_history"],
                      outputs["history_input"], outputs["history_output"])
tets, bary = v.to_output(tets, bary)      # int64 (n,), float64 (n, 4): -1 and NaN for no tet
xyz, region = v.map_positions(xyz)        # float64 (n, 3); direction="to_input", regions=...
```

`wm.VolumeTracking(history, volume_history)` maps positions only, and
`wm.VolumeTracking.load(...)` takes the four files instead.

`wm.CurveTracking` does the same for a curve history: segments of the input curve files `(n,)`
and coordinates `(n, 2)` to two vertices of the output mesh `(n, 2)`, and back, and
`feature_points()`. Ids are `(n,)` faces or segments, or `(n, 3)` / `(n, 2)` vertices on a side
named by a mesh's vertices (`output_by_vertices`, `input_by_vertices`); `warnings=True` adds the
call's warnings as a third element. Building a map holds the process-wide lock, as its loading
logs; mapping releases the GIL and changes nothing -- a `VolumeTracking` only the operation maps
it keeps, under a lock of its own -- so threads may share one object.
`docs/tracking.md` has the details.

## Shared file utilities

`io::read_mesh(path)` reads MSH, OBJ, OFF, STL and the existing libigl surface formats.
It preserves row identity and isolated vertices; cleaning belongs to component preprocessing.
Nonstandard MSH node tags are preserved as zero-based source IDs in `vid`, unless an explicit
`vid` field is supplied.
OBJ polylines retain orientation, and UV/normal values are represented per triangle corner
(`uv`: six columns; `normal`: nine columns), read only when every face corner references a value.
Existing component-specific VTU input remains
unchanged; the shared utilities do not add a VTU reader.

`io::write_mesh(path, data)` writes MSH or delegates VTU output to the existing
`paraviewo::VTUWriter`, without a new serializer or custom VTU metadata. VTU attributes use the
existing Float64 representation, with exact numeric conversion required. Attribute labels,
two-column fields (which the writer pads), names requiring XML escaping, and differing attribute
schemas across cell blocks are rejected. The writer rounds values below 1e-16 in magnitude to
zero, as in every component's own VTU output; this is reported as a warning. Use MSH when those
fields need to be preserved. MSH fields use
reversible scalar splitting with type/shape/label metadata in their data headers. Integer values
that cannot be represented exactly by MSH's Float64 field values are rejected. Physical groups
retain overlapping memberships. Undefined floating diagnostics are encoded reversibly in MSH
metadata. Unsupported cell types, encodings, and unrepresentable data throw rather than being
silently dropped.
