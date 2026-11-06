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
`std::runtime_error` before computation; this interface does not transfer arbitrary user
attributes through topology changes.

Mesh fields come in two kinds. **Data** is what a component reads: `tags`, the label columns of a
cell block, on every route; the `envelope` mesh; and a short list of named fields
(`io::ComponentIO::data_fields`), each read by a component: vertex `vid` and `corr_input_vid`,
an offset's correspondence (simwild, so topological_offset carries it, and prismatic_mesh);
vertex `labels` and tet `offset_tag` and `tag_0` (prismatic_mesh); vertex `is_cone` and triangle
`dofs` (c1_simplification); `edge_id` and `vertex_id` (isotropic_remeshing's `features`); and
triangle `uv` and `normal`, per corner (isotropic_remeshing's `feature_fields`, tracking).
Besides them, simwild reads 2D label images, cell fields `tag_<k>`; isotropic_remeshing the
triangle fields its `feature_fields` names; tracking's transfer every triangle field. Every other
field is an **annotation**, written for people -- tetwild's `energy`, triwild's `quality`, the
inputs' winding numbers `wn_<name>`, the offset's diagnostics: every reader drops it, on both
routes, and names the fields it dropped once per read, at info. So a result goes into the next
component as it is. A field a component requires (c1's `dofs`) is still an error when missing.
Until 2026-11 each reader refused, with `Unknown mesh attribute`, every field it did not list.

isotropic_remeshing takes the triangle fields its `feature_fields` names, as on the `.msh` route:
the per-face ones it keeps as labels through the remeshing and returns on `mesh`, each in the type
it was given in, with its labels (an Int64 one within 2^53 in magnitude, which Float64 holds
exactly); `uv` and `normal`, per corner (6 and 9 columns), it neither carries nor returns: their
seams are curves, in `edges`. So `mesh` goes back in with the same per-face fields named again;
not named, they are dropped, and with them their curves.

| Component | Mesh selectors | Required data |
| --- | --- | --- |
| `qslim` | `input` | Triangle geometry, no attributes |
| `shortest_edge_collapse` | `input` | Triangle geometry, no attributes |
| `isotropic_remeshing` | `input`; optional `features` | Triangle geometry and the triangle fields `feature_fields` names; `features` on the rows of `input`: edges with Int64 `edge_id`, points with Int64 `vertex_id`, or Int64 vertex `vertex_id` (-1 for none) |
| `tetwild` | `input`; optional `envelope` for tagged input | Triangle surfaces, and optionally curves and points (each `input` entry what its blocks make it: triangles, else edges, else points), no attributes. Or one tagged tet mesh, continued from as it is: tagged faces, feature curves and points, no tet tags but `ambient` |
| `triwild` | `input`; optional `envelope` for tagged input | Curves, and optionally points, no attributes. Or one tagged triangle mesh, continued from as it is: tracked curves and points, no triangle tags but `ambient` |
| `simwild` | `input`; optional `envelope` for tagged input | Surfaces (3D) or curves (2D), and optionally 3D curves and points, or tagged triangle/tet mesh |
| `topological_offset` | `input`; optional `envelope` | Tagged triangle/tet mesh |
| `manifold_extraction` | `input`; optional `envelope` | Tagged triangle/tet mesh |
| `prismatic_mesh` | `input` | Tet construction mesh with `corr_input_vid` and cell region memberships |
| `c1_simplification` | `surface_mesh`, `uv_mesh` | Surface triangle `dofs` (Float64, 36 columns), vertex `is_cone` (UInt8, 1 column); UV triangle geometry |
| `tracking` | none for `info` and `features`; `transfer`: `input`, optional `tet_mesh`; `volume`: optional `tet_mesh` and `output_tet_mesh` | The history as bytes (option `history`, not a mesh), and for `volume` the volume history as bytes (option `volume_history`). `transfer`: the triangle surface the history starts from, whose triangle attributes it carries (`uv`, `normal` per corner, any other per face), its vertex attributes dropped; the tet mesh the history ends at. `volume`: the tet meshes the volume history starts and ends at, with their `tags` if they have them |

For tagged inputs, `tags` on the top-dimensional cells names their regions, such as `tag_0`. A
column labelled `ambient` is the background, as is a cell without a tag; a mesh without a `tags`
field is all background. Tagged results label their background `ambient` explicitly; the shared
readers and writers infer nothing from an untagged row. As on the `.msh` route, the lower-dimensional blocks are tags, one `tags` column per input: in a tet mesh,
triangles are tagged faces (an input surface, each row oriented as its input) and edges tagged
feature curves (a curve input, named as every input is); in a triangle mesh, edges are tagged curves
(an input curve, each row oriented as its input). simwild, tetwild and triwild write them so, and
simwild, tetwild and triwild read them back as the tags they hold. tetwild takes a tet mesh as its
only input on the tagged route, its tets kept: no simplification, no insertion, no input winding
numbers. It holds no volumes, so a mesh whose tets carry a tag other than `ambient` is refused
(simwild remeshes those), and its point block is input points, anchored where they are. triwild
takes a triangle mesh the same way, holding its tracked curves and points and no regions: a mesh
whose triangles carry a tag other than `ambient` is refused. tetwild writes no tet tags and triwild
no triangle tags but `ambient`: regions are simwild's. Rows labelled `EnvelopeSurface` (in a triangle mesh,
an edge block without `tags` too) are the envelope instead, which cannot be combined with an
`envelope` input. Other lower-dimensional blocks are rejected.

The `envelope` input is a mesh of the call on the memory route and the path of a mesh file on the
file route (relative to `input_dir`): the `envelope` a producer returns or writes (below). It is the
reference the first component held its tags to, its input, labelled as its mesh labels them, so a
chain stays within one eps of that input instead of drifting one eps further at every stage.
simwild, given it with a tagged mesh, builds every envelope, tube and anchor around it at its own
eps, matching the envelope's labels to the mesh's face tags (2D: edge tags), regions and feature
curves by name; it throws for a label the mesh holds that the envelope lacks, and, before a
remeshing, unless every held face and feature edge received lies, exactly, in the envelopes of its
labels -- the error names the labels, the number outside and the eps. tetwild's and triwild's tagged
routes hold to it the same way, by the same code, before every run. topological_offset and
manifold_extraction carry it, not yet holding to it. Each returns it unchanged as its own
`envelope`. An `envelope` without labels -- bare geometry -- is held to as a whole, as before.

The prismatic adapter accepts scalar `tag_0` and `offset_tag`, or the corresponding columns of
`tags`. `vid` defaults to vertex row IDs; absent `labels` are derived from input-volume cells and
correspondence. `corr_input_vid` names a source `vid`, with -1 for no correspondence. Enable `save_offset_correspondence` and disable offset
optimization to obtain construction correspondence for the offset-to-prismatic workflow.

C1's 36 DOF columns flatten its 12-by-3 face matrix in row order. UV vertices use `V`'s first two
columns; UV faces correspond to surface face rows. Its unfinished tetrahedral mapping route is
unchanged.

## Results and compatibility

`mesh` is the primary output key. tetwild, triwild, simwild, topological_offset and
manifold_extraction return nothing `mesh` already holds: their tracked surfaces and curves are
its triangle and edge blocks, and the boundary of a region -- simwild's interfaces, an offset's
front, the surface manifold_extraction extracts -- is where the cell tags differ across a face
(2D: an edge), as the next component reads it. Until 2026-11 they also returned these as
`surface`, `edges` (simwild 3D: where the interfaces are not manifold), `offset` and
`input_complex`, each a second copy of the mesh's vertices. The keys besides `mesh`:

- `edges`: isotropic_remeshing's feature edges, with `features` or `feature_fields` (its Int64
  `edge_id`, and `mesh`'s Int64 `vertex_id` and per-face fields -- in the types they were given
  in, with their labels -- are what its `.vtu` output holds, an edge block of none where there are
  none), which can be the next call's `features`: a triangle mesh holds no edge block.
- `offset`: prismatic's classified offset faces.
- `envelope`: what tetwild, triwild and simwild held their tags to, the reference -- the input,
  or the simplified input under `envelope_reference` "simplified" -- labelled as `mesh` labels the
  tags: in 3D a triangle block with one `tags` column per input surface's face tag or region, and
  an edge block of the feature curves, one column per curve input; in 2D an edge block, one column per input
  curve's edge tag or region; the input points as a point block without tags, as `mesh` writes
  nothing for them. No eps: a receiver brings its own. A component given an `envelope` returns that
  one, unchanged. simwild's tag operations hold nothing to one and do not read it: they move the
  boundaries between regions, which no envelope given covers, and return one built around their
  result, each face between regions (2D: edge) labelled by the regions it bounds and its face tags,
  with the feature curves.
- `features`: tetwild's and triwild's tracked curves and anchored points, with input points.
- `uv`: c1_simplification's UV mesh.
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

isotropic_remeshing's `edges` shares the `mesh` vertex rows: its connectivity indexes the same
`V`, with the same vertex fields, so row `i` is the same vertex in both, and vertices it does not
reference are kept. `envelope`, `features` and prismatic's offset faces have vertices of their own.

topological_offset's result -- `mesh`, the `.msh`, and the `save_vtu` file -- carries the tags,
the data fields (`vid`, `labels`, `offset_tag`; in `mesh` and the `.msh`, with
`save_offset_correspondence`, `corr_input_vid`) and two annotations, `amips` per tet and
`distance_over_target` per vertex. Its `DEBUG_output` frames carry every diagnostic field.
manifold_extraction, which returns its mesh through the offset's writer, returns it so too.

The exporters' legacy file names map to these keys through one explicit table
(`io::default_output_keys`); an unmapped export or two exports with one key is an error, never
a silently dropped output. Reports contain the existing
algorithm counts, quality measures, and timings; prismatic reports also contain hybrid and
smoothing/remeshing reports. Report values are available without a report filename.

The existing topological-offset driver currently refuses 2D inputs; the memory overload preserves that restriction.

The file-oriented entry points retain existing configurations, relative-path conventions,
output filenames, intermediate exports, and report files. The tagged components (simwild, triwild,
topological_offset, manifold_extraction) build the same result on both routes: the file route
writes its `mesh` through `io::write_mesh` and its `envelope`, when it has one, as a file of its
own, `<output>_envelope.msh` (the mesh file holds the mesh, which gmsh and polyfem read), and
reads its `.msh` input, and an `envelope` file, through `io::read_mesh`. An explicit per-call output policy
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
