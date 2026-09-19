# Prismatic mesh input

Run from the repository root:

```sh
cmake --build build --target wmtk_app -j 4
./build/app/wmtk_app -j mydata/prismatic_mesh.json
```

The JSON `input` and `output` paths are relative to the JSON file's directory. Example:

```json
{
  "application": "prismatic_mesh",
  "input": "deliverable/cube/cube_2_construction_correspondence.vtu",
  "output": "resultmesh.vtu",
  "thicknessratio": 0.1,
  "iterations": 5,
  "min_tet_volume": 1e-12,
  "smoothing_max_backtracks": 40
}
```

The pipeline loads a tetrahedral construction mesh and its correspondence, keeps only the
offset band tetrahedra (`offset_tet_tags == 1`), computes targets, optimizes and writes the
processed mesh to `output`.
It does not yet construct prisms. A filename without an extension gets `.vtu` appended.
For this example, the output is `mydata/resultmesh.vtu`.
`load_prismatic_mesh(path)` returns a
`PrismaticMeshInput` containing WMTK `TetMesh` connectivity, vertex coordinates,
tetrahedra, and the following required VTU fields:

| Field | Association | Meaning |
| --- | --- | --- |
| `labels` | PointData | 0 = other, 1 = input surface, 2 = offset surface |
| `vid` | PointData | Unique original vertex ID |
| `corr_input_vid` | PointData | Original input vertex ID for every offset vertex; -1 elsewhere |
| `tag_0` | CellData | Input volume membership (0/1) |
| `offset_tag` | CellData | Offset band membership (0/1) |

`offset_tag` is intentional: `offset` is zero in the construction export.
On loading, VTU `labels` becomes `vertex_tags` (1 = input, 2 = offset, -1 = other),
and VTU `offset_tag` becomes `offset_tet_tags` (1 = band, -1 = other).
The source VTU uses 0 for these other vertices/cells; it is converted to -1 in memory.
`keep_offset_band(input)` filters tetrahedra and cell attributes together, preserving their
order and tag values, and rebuilds the WMTK mesh. Vertex arrays, IDs, tags and correspondence
remain unchanged; vertices unused by the band are inactive in the rebuilt mesh.
The component entry point in `prismatic_mesh.cpp` handles configuration and loading, then
calls `prism_main(input)` in `prism_main.cpp`, where the algorithm pipeline continues.
After filtering, `label_offset_faces(input)` labels each unique face whose three vertices
have `vertex_tags == 2`. `offset_face_tags[face.fid(*input.mesh)]` is 1 for three distinct
`corr_input_vid` values (bijective), 2 for exactly two equal values, 3 for all equal,
and -1 for non-offset faces. The vertex rule applies to all faces, not just boundary faces.
Face tags must be recomputed after topology or correspondence changes.

`evaluate_target_positions(input, thicknessratio)` next computes targets without moving
the mesh. `thicknessratio` defaults to 0.1 and must be finite and positive. Its length scale
is the mean length of **unique input surface edges** in the band mesh (edges of faces whose
three vertices have tag 1), not the mean over ambient or input volume diagonals. The target is
`p + optimal_normal * thicknessratio * input_average_edge_length`.

Active offset vertices form a graph using mesh edges with two offset endpoints and equal
source `corr_input_vid`. Connected components never traverse an input vertex or a vertex
with a different correspondence. Each component appears in `offset_components`, and
`input_to_components[input_row]` indexes the components belonging to that input vertex.
`vertex_component_ids` assigns a component to each active offset vertex; other entries are -1.

Normals come from input faces incident to the corresponding input vertex. Input faces
separate sectors in that vertex's tetrahedral one-ring. For each sector, input face normals
are oriented into its band tetrahedra and assigned to the correspondence components touching
that sector. This preserves opposite normal orientations for a two-sided sheet. A component
touching both sides inherits both constraints rather than arbitrarily flipping one side.

For unit face normals, solve `min_d sum_f (n_f.dot(d) - 1)^2`, then normalize `d`.
The solver forms an orthonormal basis of the normal span (rank tolerance 1e-10) and solves
the reduced normal equations using Eigen dense `FullPivLU` (relative pivot threshold 1e-12).
This returns a minimum-norm LS direction without regularization and supports planar rank-one
and crease rank-two normal sets. Every `n_f.dot(d)` must exceed 1e-8 after normalization.
Missing normals, degenerate supporting geometry, unavailable thickness scale, or a failed
solve/validity check mark the whole component singular. This is a conservative LS validity
test, not an independent proof that no direction could satisfy the original constrained problem.

Each component stores `optimal_normal`, `target_position`, `singular`, and `singular_reason`.
Each member vertex gets the same valid normal and target. For singular vertices the normal is
zero and the target array keeps the current position as an **invalid placeholder**; later
optimization must branch on `singular_vertex_tags` (1 singular, 0 valid, -1 non-offset/inactive).
The optimization below deliberately freezes these initial targets and component IDs.
Recomputing them is necessary only when deliberately changing the reference correspondence
or reference geometry, not during this fixed-target optimization.

`optimize_prismatic_mesh` runs `iterations` rounds (default 5), each in the order
**collapse → unlock → smoothing**. Set
`iterations` to 0 to export the initial band and target fields without moving/collapsing it.
Each round attempts edges whose endpoints are both offset vertices in the same component
and with the same correspondence. A successful collapse keeps one endpoint's position,
source ID, correspondence and component. The endpoint nearer the fixed target is preferred;
if that directed collapse fails, the other direction is tried. Newly affected edges are
queued for another attempt within the same collapse pass.

Every collapse must pass WMTK's boundary-aware link condition and connectivity checks.
Before changing connectivity, the tetrahedral stars of **both** endpoints are checked, as
are the predicted retained cells. Tets containing the collapsed edge are deleted rather
than kept as degenerate cells. Rejected operations leave geometry and attributes unchanged.
The actual survivor star is checked again before external metadata is committed.

Smoothing processes each active offset vertex sequentially, trying its component's fixed
target first (`alpha=1`), then `alpha=1/2,1/4,...` until all incident tetrahedra pass. It tries
at most `smoothing_max_backtracks` halvings (default 40, range 0--60); exhaustion leaves the
vertex unchanged. Singular components are eligible for safe collapse but skip this smoothing
until a separate singular optimization rule is implemented. Smoothing changes no connectivity,
so it preserves the existing link relations. Each tet determinant is affine along a single-
vertex move, making a segment between valid endpoints safe as well.

All operations require the **signed** volume `det(p1-p0,p2-p0,p3-p0)/6` to be strictly
greater than `min_tet_volume` (default 1e-12, finite and nonnegative). The units are coordinate
units cubed; this is an absolute volume floor, not a normalized shape-quality metric.
GMP rational arithmetic evaluates the comparison exactly on the stored double coordinates,
including the threshold. Optimization rejects an initial band with any cell already at or
below the floor, identifying its tet ID, before performing any operation.

Unlock identifies a tau22 tetrahedron by its two input vertices (source IDs `a`, `b`)
and two offset vertices with distinct correspondences exactly matching `a`, `b`.
Input vertices' own source IDs are used here, since their `corr_input_vid` is `-1`.
For each tau22, first split `(input a, offset b)` at its midpoint `x`, then collapse
`x → offset a`. If that attempt fails, try the symmetric split `(input b, offset a)`
and collapse `x → offset b`. The collapse always keeps the original offset endpoint.

The entire split/collapse pair is atomic, using a temporary mesh until both operations
succeed. Every split child must exceed the volume floor; the subsequent collapse passes
the same link, inversion and volume checks as ordinary collapse. Failure leaves the
original mesh and attributes unchanged. Successful unlock changes connectivity only:
all original vertices keep their positions, labels, correspondence, component IDs, normals
and targets, and no midpoint remains. Split children inherit their parent's cell tags.
Each pass visits its starting tau22 candidates once, trying both directions when needed;
newly created tau22 cells wait for the next iteration. Unsafe tau22 cells may remain.
Logs and `optimization_iterations[i].unlock` report starting candidates, attempted cells,
successful operations and remaining tau22 cells.

The optimizer preserves vertex row indices and original IDs. After each accepted collapse,
it rebuilds compact live cell arrays and WMTK connectivity together, and updates component
membership and reverse correspondence lists. It never calls `consolidate_mesh()` to renumber
vertices. Existing face tuples/IDs are invalidated by these rebuilds; face classification is
recomputed after optimization. Iteration counts are recorded in `optimization_iterations`.
`write_vtu.cpp` exports only active vertices and retained tetrahedra. Output connectivity
uses compact row indices; `vid` and `corr_input_vid` preserve the original source IDs.
The output fields are `labels` (1 = input, 2 = offset, -1 = other), `vid`,
`corr_input_vid`, `tag_0` and `offset_tag` (1 = band, -1 = other).
Volume CellData also includes `tau22` (1 = tau22, -1 = other), evaluated on the final mesh.
The loader accepts both the construction file's 0 and the output's -1 for other elements.
Face classification is exported separately as `<output_stem>_offset_faces.vtu`, a triangle
mesh with `offset_face_tag` in CellData and source IDs/correspondence in PointData.
For the example this is `mydata/resultmesh_offset_faces.vtu`; color by `offset_face_tag`
to inspect the three regions. The volume output retains its tetrahedral cell tags.
Both VTUs also export `component_id`, `singular_component`, `optimal_normal` and
`target_position` as PointData. The mesh Points contain the optimized positions; the target
and normal fields retain their initially computed values.
`corr_input_vertex` resolves source IDs to mesh row indices, and
`input_to_offset_vertices[input_row]` contains all corresponding offset rows.
Several offset vertices may correspond to one input vertex. Positions and attributes
are owned by the returned input object alongside `TetMesh`. The component's optimizer keeps
them synchronized; direct external WMTK operations do not automatically update these arrays.

Supported VTU: a single UnstructuredGrid piece with four-node tetrahedra, ASCII or
uncompressed inline base64 arrays, UInt32/UInt64 binary headers and either byte order.
A missing byte_order defaults to LittleEndian for the existing paraviewo exports.
Compressed, appended, multi-piece and mixed-cell files are rejected explicitly.
Missing fields, invalid indices and invalid correspondence also cause an error.

Tests:

```sh
cmake --build build --target wmtk_test_prismatic_mesh -j 4
ctest --test-dir build -R '^wmtk_test_prismatic_mesh$' --output-on-failure
```
