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
require `application`, `input_dir`, or output filenames. The existing Python dispatcher still
uses the file-oriented interface; bindings for MeshData are not part of this change.

```cpp
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/components/qslim/qslim.hpp>

wmtk::MeshMap meshes{{"surface", wmtk::io::read_mesh("input.obj")}};
auto [outputs, report] = wmtk::components::qslim::qslim(
    {{"input", "surface"}, {"target_rel", 0.5}}, meshes);
wmtk::io::write_mesh("simplified.vtu", outputs.at("mesh"));
```

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
  finite. Attributes may contain undefined numerical diagnostics such as NaN.

Only top-dimensional connectivity is needed to construct an algorithm's mesh. Explicit lower
blocks identify the edges/faces carrying attributes; their row order is independent of internal
mesh IDs. Do not use internal edge/face IDs as interchange indices. Exact positions, caches,
rollback buffers, and mesh objects are not part of MeshData.

## Input contracts

Selectors retain their component-specific JSON fields. A single selector is a dictionary key;
ordered selector arrays preserve input order and existing `input_names` / transform semantics.
Unselected dictionary entries are not validated. Missing keys and incompatible inputs throw
`std::runtime_error` before computation. Unknown attributes are rejected; this interface does
not transfer arbitrary user attributes through topology changes.

| Component | Mesh selectors | Required data |
| --- | --- | --- |
| `qslim` | `input` | Triangle geometry, no attributes |
| `shortest_edge_collapse` | `input` | Triangle geometry, no attributes |
| `isotropic_remeshing` | `input` | Triangle geometry, no attributes |
| `tetwild` | `input`; `input_edges`, `input_points` | Triangle surfaces; optional curve/point geometry, no attributes |
| `triwild` | `input`; `input_points` | Curve geometry, optional free points, no attributes |
| `simwild` | `input`; optional `envelope` for tagged input | Surface/curve geometry, or tagged triangle/tet mesh |
| `topological_offset` | `input`; optional `envelope` | Tagged triangle/tet mesh |
| `manifold_extraction` | `input`; optional `envelope` | Tagged triangle/tet mesh |
| `prismatic_mesh` | `input` | Tet construction mesh with `corr_input_vid` and cell region memberships |
| `c1_simplification` | `surface_mesh`, `uv_mesh` | Surface triangle `dofs` (Float64, 36 columns), vertex `is_cone` (UInt8, 1 column); UV triangle geometry |

For tagged inputs, `tags` is required on the top-dimensional cells. Its labels name regions
such as `tag_0`; `ambient` is the untagged region. Existing exported diagnostics have fixed allowed
names in the adapters and may be supplied again. Extra unrelated fields are errors.

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
geometry under keys such as `surface`, `uv`, `edges`, `offset`, `envelope`, and `features`.
Manifold extraction returns its extracted boundary as `surface`, retaining an input-complex
preview as `input_complex` when available. Prismatic `export_input_and_shell` adds `input_and_shell`. Reports contain the existing
algorithm counts, quality measures, and timings; prismatic reports also contain hybrid and
smoothing/remeshing reports. Report values are available without a report filename.

The existing topological-offset driver currently refuses 2D inputs; the memory overload preserves that restriction.

The file-oriented entry points retain existing configurations, relative-path conventions,
output filenames, intermediate exports, and report files. An explicit per-call output policy
turns off their file observers for memory calls. Component entry points serialize their use of
the legacy process-wide loggers, temporarily using console sinks during memory calls and restoring
the previous logger afterward. Existing algorithm options, filtering,
cleaning, and exact-arithmetic decisions are shared between the two routes.

## Shared file utilities

`io::read_mesh(path)` reads MSH, OBJ, OFF, STL and the existing libigl surface formats.
It preserves row identity and isolated vertices; cleaning belongs to component preprocessing.
Nonstandard MSH node tags are preserved as zero-based source IDs in `vid`, unless an explicit
`vid` field is supplied.
OBJ polylines retain orientation, and UV/normal values are represented per triangle corner
(`uv`: six columns; `normal`: nine columns). Existing component-specific VTU input remains
unchanged; the shared utilities do not add a VTU reader.

`io::write_mesh(path, data)` writes MSH or delegates VTU output to the existing
`paraviewo::VTUWriter`, without a new serializer or custom VTU metadata. VTU attributes use the
existing Float64 representation, with exact numeric conversion required. Attribute labels,
two-column fields (which the writer pads), values it would round to zero, names requiring XML
escaping, and differing attribute schemas across cell blocks are rejected. Use MSH when those
fields need to be preserved. MSH fields use
reversible scalar splitting with type/shape/label metadata in their data headers. Integer values
that cannot be represented exactly by MSH's Float64 field values are rejected. Physical groups
retain overlapping memberships. Undefined floating diagnostics are encoded reversibly in MSH
metadata. Unsupported cell types, encodings, and unrepresentable data throw rather than being
silently dropped.
