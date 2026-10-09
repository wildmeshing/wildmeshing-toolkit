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
  "keep_background_mesh": false,
  "prism_dominant": true,
  "min_tet_volume": 1e-12,
  "smoothing_max_backtracks": 40
}
```

The input can also be a **Gmsh 4.1 `.msh` file**, in ASCII or native-endian binary
with 8-byte data sizes. For the Thingi10K construction sample, set the input directly:

```json
"input": "thingi10k_offset_construction_sample20/models/55928/out.msh"
```

No VTU conversion is needed. MSH input must contain only four-node tetrahedra and a
scalar `corr_input_vid` NodeData field covering every node. For construction exports,
the reader follows volume physical group **names**, through their entity associations:
`tag_0` = input solid, `offset` = offset band, `ambient` = background. The aliases
`input`, `offset_band`, and `background` are supported too; physical group numbers
are not hard-coded. Input vertex labels are derived from input tets, offset vertices
are identified by nonnegative correspondence, and other vertices are background.

Without a `vid` NodeData field, a node's source ID is **its Gmsh node tag minus one**,
matching the construction export's zero-based `corr_input_vid`. With `vid`, correspondence
refers to that explicit source ID instead. The reader never treats correspondence values
as file row indices. Noncontiguous tags and reordered node/element data are supported;
coordinates and tet corner order are preserved without rescaling or reorientation.

Pure-tet PRISM MSH exports are also readable: explicit scalar `labels`/`vid` NodeData
override inference, and the pair of `tag_0`/`offset_tag` ElementData fields override
physical group inference. Each supplied field must cover all nodes or all elements once.
Missing or invalid correspondence, ambiguous regions, duplicate IDs/tets and unknown
connectivity references are rejected. Mixed tet/prism/pyramid output must be decomposed
before loading as optimization input, just as with VTU.

The pipeline loads a tetrahedral construction mesh and its correspondence, computes targets
on the offset band (`offset_tet_tags == 1`), then optimizes offset vertices and writes the
**original input volume plus the optimized offset band** to `output`. Input volume cells
(`tag_0 == 1`) outside the band are saved before optimization and reattached afterwards.
Their connectivity is
unchanged, and all input vertices retain their original coordinates and source IDs. The input
volume and band share the same vertex rows at their interface.

By default, exterior background cells are omitted. Set `"keep_background_mesh": true` to
retain these cells and include them in the output. With background remeshing disabled, the active optimization mesh contains the
band plus background tetrahedra incident to an offset vertex, preserving the complete stars
needed for signed-volume and topology checks. Accepted topology changes update these incident
background cells. Other background cells are stored separately, checked once for positive
volume, and reattached unchanged alongside the input volume for output. Cutting away
cells with no offset vertex creates no new boundary face incident to an offset vertex.
Input vertices remain fixed. Background-only vertices also remain fixed unless the optional
background remeshing stage below is enabled. Components, normals, thickness
scale and fixed targets are computed on the band alone in both modes, so the switch compares
the effect of retaining the exterior constraints with the same target field. Background cells
must keep strictly positive volume; `min_tet_volume` applies to shell operations, newly
introduced reconstruction tets and candidate pyramid decompositions. The logs report
the optimization domain and per-iteration collapse, unlock and smoothing counts. Use distinct
output names for the two runs when comparing results.

Enable adaptive background remeshing with:

```json
"keep_background_mesh": true,
"background_remeshing": true,
"background_remeshing_passes": 2,
"background_quality_threshold": 0.1,
"background_remeshing_max_operations": 500,
"background_remeshing_max_attempts": 20000
```

This stage runs after each iteration's offset collapse/unlock/smoothing, followed by another
collapse pass. It selects background tets with **mean ratio** below the quality threshold,
and regions blocking a directed offset collapse. A repair target must pass the existing
shell-volume, orphan-vertex and link checks; at least one retained background tet must be
predicted to become nonpositive. Shell-floor failures are therefore excluded from these targets.
Mean ratio is `12 * (3 * signed_volume)^(2/3) / sum(squared_edge_lengths)`, with 1 for a regular
tet and 0 for a degenerate tet; this metric is invariant under uniform scaling.

The available operations are pure-background **2-to-3 face swaps**, **3-to-2 edge swaps**,
and backtracked interior-vertex relaxation. Vertex count, source IDs and correspondence are
preserved. Input and band connectivity/coordinates stay fixed during this stage, as do all
vertices on the exterior boundary. Enabled runs retain the entire background in the active
mesh so movable vertices have complete incident stars. This uses more memory than the default
offset-neighborhood domain. All new/moved background tets must pass the exact strictly-positive
volume predicate; background has no absolute volume floor. Quality operations strictly improve
the worst mean ratio in their cavity. Targeted repairs strictly reduce the cavity's collapse
blockers and retain at least half its previous worst mean ratio. Interior moves are bounded to
a quarter of their mean incident edge length and backtrack at most 12 times.

Operation and attempt budgets apply to each outer optimization iteration, across all local
passes. When repair targets exist, the quality phase leaves half the remaining budgets for
targeted repair. Budgets bound work; some bad tets or blocked edges may remain. The logs and
`<output_stem>_background_remeshing.json` report swaps, moves, low-quality counts before/after,
budget exhaustion, and collapses accepted by the retry. `rescued_background_blocked_collapses`
counts only accepted directions identified as background-blocked immediately before remeshing;
`additional_collapses_after_remesh` includes all retry successes. Hybrid reports include the
same diagnostics. The default `background_remeshing=false` preserves the existing algorithm.

Set `"jacobian_smoothing_iterations": 20` to enable a separate prism repair stage after
tet collapse/smoothing and before hybrid reconstruction. Its default is **0 (disabled)**.
The stage keeps input positions, connectivity and correspondence fixed and moves only offset
vertices in matched prism columns, including columns with negative Jacobians. It does not
require a valid target normal, so offset vertices skipped as singular by target smoothing
can participate here. Each move uses only the vertex's incident prisms and tetrahedra.

The local position problem is a convex squared-hinge QP: minimize deficits below
`jacobian_target` (default **0.01**, all sample weights are 1), plus
`jacobian_position_weight` (default **0.001**) times squared displacement from the position
at the start of this stage. Jacobians are divided by a fixed per-prism scale: twice the input
triangle area times the initial mean column length. Position displacements are divided by
the target thickness, or the initial column length if no target thickness is available.
The displacement penalty and bounds limit geometry and thickness changes; this stage does
not impose exact offset distance or a separate global collision constraint.

Slacks are eliminated as `max(0, target-J)`; a piecewise-quadratic Newton method with an
active-set constrained QP and line search solves each three-coordinate update. Hard
halfspaces protect incident tet volumes and already valid prisms. `jacobian_max_step_ratio`
(default **0.25**) and `jacobian_max_displacement_ratio` (default **0.5**) bound single moves
and total displacement, relative to the above thickness scale. Inscribed axis-aligned boxes
make these bounds linear and ensure the corresponding Euclidean displacement limits.
Accepted moves must reduce the actual Jacobian penalty and the penalty plus displacement
regularization. Therefore the positive target is a soft goal, not a mandatory final value.

Every trial checks the analytic side-edge Jacobian minima of **all** incident prisms, and
recomputes the exact tet volume predicate on the stored coordinates. Existing bad prisms may
improve gradually; already positive prisms must remain positive. New worst reference points
can be added to the local solve before retrying. Modified shell tets must exceed
`min_tet_volume`, while background tets only require positive volume. Failed local solves
leave the vertex unchanged; phase-I/search budget failures are reported without claiming
mathematical infeasibility. A round with no accepted moves stops this stage.

The hybrid report's `jacobian_smoothing` object contains options, initial/final metrics,
per-round progress and rejection categories, every moved vertex's old/new position and
column length, and all candidate prisms' normalization scales and minimum Jacobians.
For a paired experiment use the same executable with iterations 0 and 20, and separate outputs.

Set `"prism_dominant": true` to convert the optimized band into a conforming mixture of
prisms (VTK wedge, type 13), pyramids (14) and tetrahedra (10). The default is false for
compatibility with the tetrahedral workflow. Conversion is a separate final stage in
`prism_dominant.cpp`; vertex positions and attributes remain intact. The band tetrahedral
connectivity is updated to the final reference splits after optimization.
A filename without an extension gets `.vtu` appended.
For this example, the output is `mydata/resultmesh.vtu`.
To export the prism-dominant volume as MSH, set `"output": "resultmesh.msh"` and
`"prism_dominant": true`. The `.msh` writer uses **Gmsh 4.1 ASCII** with native linear
tetrahedra (Gmsh type 4), prisms (6) and pyramids (7). Coordinates are written with
round-trip double precision. Three volume physical groups identify `input` (1),
`offset_band` (2) and `background` (3), when present. Vertex correspondence and cell
attributes are preserved as NodeData and ElementData. Mesh node/element tags are
one-based export IDs; `vid` and `corr_input_vid` still contain the original source IDs.
The offset-face companion remains `<output_stem>_offset_faces.vtu`, and hybrid conversion
still writes `<output_stem>_hybrid.json`. Both `.vtu` and `.msh` also support pure tet output
when `prism_dominant` is false.

Set `"export_input_and_shell": true` to additionally write
`<output_stem>_input_shell.msh` and `<output_stem>_input_shell.vtu`. Both contain the **union
of input volume and shell** (`tag_0 == 1` or `offset_tag == 1`), with background cells and
unused background-only points omitted. These are filtered exports of the same final result;
coordinates, mixed-cell types, correspondence and reference decompositions are preserved.
All input points are retained, including isolated input points. The extra hybrid report records
the filtered cell counts; `source_tet_ids` continues to use rows in the full reconstructed
reference mesh. The main output is still written as configured. The option defaults to false.

`load_prismatic_mesh(path)` dispatches by `.vtu`/`.msh` extension and returns a
`PrismaticMeshInput` containing WMTK `TetMesh` connectivity, vertex coordinates,
tetrahedra, and the following required VTU fields (inferred or read as described above for MSH):

| Field | Association | Meaning |
| --- | --- | --- |
| `labels` | PointData | 0 = other, 1 = input complex (including volume-interior vertices), 2 = offset surface |
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
The loaded input-vertex count includes those inactive vertices. Compare component coverage
against the **active input vertices in the band**, not against the full-file input count.
The logs report both counts and the number of active input vertices with zero, one or multiple
components; two-sided sheets can have more than one component per input vertex.
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
`iterations` to 0 to export the input volume, initial band and target fields without
moving/collapsing them.
Each round attempts edges whose endpoints are both offset vertices in the same component
and with the same correspondence. A successful collapse keeps one endpoint's position,
source ID, correspondence and component. The endpoint nearer the fixed target is preferred;
if that directed collapse fails, the other direction is tried. Newly affected edges are
queued for another attempt within the same collapse pass.

Every collapse must pass WMTK's boundary-aware link condition and connectivity checks.
Before changing connectivity, the tetrahedral stars of **both** endpoints are checked for
positive volume. Retained shell cells whose connectivity changes must exceed the operation's
volume floor; background cells only need positive volume, even when changed. Unchanged shell
cells may already be smaller. Tets containing the collapsed edge
are deleted rather than kept as degenerate cells, and may also have started below the floor.
Rejected operations leave geometry and attributes unchanged. The actual survivor star is
checked again before external metadata is committed, applying the floor only to changed shell cells.

Smoothing processes each active offset vertex sequentially, trying its component's fixed
target first (`alpha=1`), then `alpha=1/2,1/4,...` until all incident tetrahedra pass. It tries
at most `smoothing_max_backtracks` halvings (default 40, range 0--60); exhaustion leaves the
vertex unchanged. Singular components are eligible for safe collapse but skip this smoothing
until a separate singular optimization rule is implemented. Smoothing changes no connectivity,
so it preserves the existing link relations. Each tet determinant is affine along a single-
vertex move, making a segment between valid endpoints safe as well.

New or modified shell tetrahedra produced by an accepted operation require the **signed** volume
`det(p1-p0,p2-p0,p3-p0)/6` to be strictly greater than `min_tet_volume`
(default 1e-12, finite and nonnegative). The units are coordinate
units cubed; this is an absolute volume floor, not a normalized shape-quality metric.
GMP rational arithmetic evaluates the comparison exactly on the stored double coordinates,
including the threshold. This is an operation acceptance condition, not a global mesh minimum:
existing positive cells at or below the floor may remain in the band or background and do not
abort the pipeline. An operation that would leave its changed shell cells below the floor is
skipped; other operations continue. Background cells only require strictly positive volume
throughout collapse, smoothing and unlock, regardless of their size. Smoothing may raise an
existing small positive shell cell above the floor.
Initial optimization and fixed-background cells are still rejected if their volume is
nonpositive or their coordinates are nonfinite.

Unlock identifies a tau22 tetrahedron by its two input vertices (source IDs `a`, `b`)
and two offset vertices with distinct correspondences exactly matching `a`, `b`.
Input vertices' own source IDs are used here, since their `corr_input_vid` is `-1`.
Two input and two offset vertices alone do not make a tau22: a normal tetrahedralization of
a prism can contain such cells with three distinct correspondence identities. The initial
band log reports both the broad 2-input/2-offset count and the stricter tau22 count.
For each tau22, first split `(input a, offset b)` at its midpoint `x`, then collapse
`x → offset a`. If that attempt fails, try the symmetric split `(input b, offset a)`
and collapse `x → offset b`. The collapse always keeps the original offset endpoint.

The entire split/collapse pair is atomic, using a temporary mesh until both operations
succeed. Every shell split child must exceed the volume floor; background split children
only need positive volume. The subsequent collapse passes
the same link, inversion and volume checks as ordinary collapse. Failure leaves the
original mesh and attributes unchanged. Successful unlock changes connectivity only:
all original vertices keep their positions, labels, correspondence, component IDs, normals
and targets, and no midpoint remains. Split children inherit their parent's cell tags.
Each pass visits its starting tau22 candidates once, trying both directions when needed;
newly created tau22 cells wait for the next iteration. Unsafe tau22 cells may remain.
Logs and `optimization_iterations[i].unlock` report starting candidates, attempted cells,
successful operations and remaining tau22 cells.

The optimizer preserves vertex row indices and original IDs. Collapse candidates, unlock
candidates and smoothing vertices are enumerated from the offset vertices and their stars.
After each accepted collapse, only the two endpoint stars are synchronized, with deleted cell
rows marked inactive; component membership and reverse correspondence lists are updated too.
Cell arrays and WMTK connectivity are compacted together once at the end of the collapse pass,
before unlock and smoothing. The standalone `try_collapse_offset_edge` API compacts before
returning. Vertex rows are never renumbered. Existing face tuples/IDs are invalidated by
topology edits and rebuilds; face classification is
recomputed after optimization. Iteration counts are recorded in `optimization_iterations`.
`write_vtu.cpp` exports retained tetrahedra, their active vertices and all input vertices
(including input points with no incident retained cell). Interior input vertices have
`component_id = -1`: components belong to offset vertices, not to the input volume.
Output connectivity
uses compact row indices; `vid` and `corr_input_vid` preserve the original source IDs.
The output fields are `labels` (1 = input, 2 = offset, -1 = other), `vid`,
`corr_input_vid`, `tag_0` and `offset_tag` (1 = band, -1 = other).
Volume CellData also includes `tau22` (1 = tau22, -1 = other), evaluated on retained tetrahedra;
prisms and pyramids have -1.
The loader accepts both the construction file's 0 and the output's -1 for other elements.
Face classification is exported separately as `<output_stem>_offset_faces.vtu`, a triangle
mesh with `offset_face_tag` in CellData and source IDs/correspondence in PointData.
It also contains the binary CellData field `non_surjective`: 1 for faces with repeated
`corr_input_vid` values (face tags 2 or 3), and 0 for three distinct values (face tag 1).
Here the flag identifies offset triangles mapping to an input edge or vertex rather than
covering an input triangle; it does not measure coverage of the entire input surface.
In ParaView, open `<output_stem>_offset_faces.vtu` and color by `non_surjective`, or apply a
Threshold on that cell field with lower and upper bounds both set to 1 to isolate these faces.
For the example this is `mydata/resultmesh_offset_faces.vtu`; color by `offset_face_tag`
to inspect the three regions. The volume output retains its tetrahedral cell tags.
The volume output and offset-face VTU also export `component_id`, `singular_component`,
`optimal_normal` and `target_position` as point/node data. Mesh coordinates contain the optimized positions; the target
and normal fields retain their initially computed values.
`corr_input_vertex` resolves source IDs to mesh row indices, and
`input_to_offset_vertices[input_row]` contains all corresponding offset rows.
Several offset vertices may correspond to one input vertex. Positions and attributes
are owned by the returned input object alongside `TetMesh`. The component's optimizer keeps
them synchronized; direct external WMTK operations do not automatically update these arrays.

## Prism dominant conversion

Conversion starts from offset triangles on the **band boundary**. Three distinct
correspondences identify a candidate input triangle, which must actually exist in the band
(including an input sheet shared by the bands on its two sides). Its six corners must contain
exactly one of the six standard three-tet
prism triangulations in the current mesh. Source tetrahedra cannot belong to two output cells.
Unmatched or overlapping candidates remain tetrahedral.

Vertices touching a repeated-correspondence boundary triangle seed the tet region. A candidate
with one affected offset corner uses a pyramid plus a tet, with the **pyramid apex always on
the input side**. Its quadrilateral base is the opposite prism side. Candidates with two or
three affected corners become three tetrahedra. Tets outside matched candidates remain fixed.
A pyramid's lateral triangular interfaces require a prism-derived transition region; a direct
interface to an unmatched original tet core expands the retained region to create that buffer.
The input triangular caps and their adjacent input-volume tetrahedra stay unchanged.

Prism admission checks the six-node wedge's **minimum signed Jacobian determinant over the
reference element**, together with the existing 21 sample points.
Using reference coordinates `r >= 0, s >= 0, r+s <= 1, 0 <= t <= 1`, the mapping is
`x(r,s,t) = (1-t)*((1-r-s)*p0+r*p1+s*p2) + t*((1-r-s)*p3+r*p4+s*p5)`.
At each of `t = 0, 0.5, 1`, the triangle's three corners, three edge midpoints and centroid
are sampled. Every sampled determinant must be finite and strictly positive; no volume floor
is imposed on the Jacobian. In addition, `det(J)` is affine in `(r,s)` on a fixed-height
triangle and quadratic in `t`, so its global minimum occurs on one of the three
input-to-offset column edges. Each edge is checked at both endpoints and any interior
quadratic minimum. This catches folds between the fixed samples; coefficient calculations
and stationary-point evaluations use floating-point arithmetic. The node ordering and interpolation follow the
[VTK six-node wedge](https://github.com/Kitware/VTK/blob/master/Common/DataModel/vtkWedge.cxx).
A prism is no longer rejected just because an unused tetrahedral split is invalid.

The actual reference tetrahedra must still all have positive signed volume. When reconstruction
changes a split, each newly introduced tet must exceed `min_tet_volume`; original unchanged
tets only need positive volume. Both candidate pyramid decompositions still require all tets
above `min_tet_volume`, using the same exact rational predicate as optimization. Templates
are oriented on reference cells; negative signs are never repaired with absolute values.
A failed prism Jacobian check adds its offset corners to the retained-region seeds and keeps
its original tets. Existing small positive band tets may remain or be grouped into a
Jacobian-valid prism without turning the operation floor into a program failure.

Prisms and prism-derived tet regions can select a new three-tet decomposition to satisfy the
input-apex pyramids. Each shared quadrilateral carries a single diagonal constraint, used on
both sides even when both output cells keep the quad. Interfaces to unmatched/fixed tetrahedra
and exterior boundary triangles keep their original triangulations. The six choices per prism
are filtered by their tet volumes, these constraints and the choices compatible with an
input-apex pyramid. A Jacobian-valid prism can therefore offer fewer than six admissible splits.
Constraint propagation followed by deterministic search selects a consistent assignment,
preferentially preserving original splits. If it cannot find one, an affected transition region
is frozen to its original tets, all three offset corners extend the tet-region seeds, and the
frontier is reconstructed. Each failed attempt freezes a new region, so fallback terminates.
Search uses at most 10,000 states per attempt; reaching that limit conservatively expands the
retained region and is recorded separately, rather than claiming the constraints are infeasible.

Output shape changes only progress from prism to pyramid+tet to all tets. Flexible prism-derived
tet regions can change their internal diagonals during reconstruction; frozen original regions
cannot. A quad facing two triangles forces its neighbor to downgrade, and this propagates until
all output interfaces agree. The final reference tetrahedral mesh is built privately and committed
to `PrismaticMeshInput` only after validation. Input and background cells, vertex coordinates,
source vertex IDs, correspondence, normals and targets remain unchanged.

The hybrid JSON report records the Jacobian sample positions, minimum sampled determinant
among output prisms, `invalid_prism_jacobian` rejections, and the histogram
`jacobian_valid_candidate_admissible_split_counts`. Validation reports sampled and global prism Jacobians,
positive reference tets, new-tet volume floors and pyramid decompositions separately.

Validation checks positive reference tets, exact coverage of the final reference mesh,
triangle/quad interface agreement, opposite face orientations, and input-side pyramid apices.
The new reference mesh must have exactly the same oriented exterior triangles as the original
volume mesh. A changed diagonal on a warped internal quad is always changed on both sides;
this may redistribute volume between adjacent candidate prisms, while preserving the boundary
of their union. No coplanarity assumption is used for shared faces.

Hybrid VTU adds these CellData fields:

| Field | Meaning |
| --- | --- |
| `cell_type` | 10 = tetrahedron, 13 = prism, 14 = pyramid |
| `prism_candidate` | Candidate index, or -1 for cells outside a matched candidate |
| `source_tet_count` | 1, 2 or 3 final reference tetrahedra represented by the cell |
| `source_tet_ids` | Up to three row IDs in the final reconstructed reference tet mesh, padded with -1 |
| `tet_decomposition` | Up to three oriented tets, each four **local corner indices** in this cell; 12 entries padded with -1 |
| `retriangulated` | 1 if the cell belongs to a prism region whose reference split changed, otherwise 0 |

The decomposition field preserves the precise prism/pyramid subdivision and the shared-face
diagonals for downstream tetrahedralization. `<output_stem>_hybrid.json` records cell counts,
validation results, rejected candidates and transition/tet regions. Its `retriangulated_regions`
records the original and new tets using source vertex IDs, along with the affected reference row
IDs. After a split changes, `source_tet_ids` refers to these final reference rows, not a claim
that the new cell is a union of those pre-reconstruction tets. In ParaView, color the
volume by `cell_type`; select `offset_tag = 1` to inspect only the reconstructed band.
MSH exports the same attributes, except that the 12-component `tet_decomposition` is split
into scalar fields `tet_decomposition_0` through `tet_decomposition_11`: Gmsh view data
supports 1, 3 or 9 components. The local corner indices remain zero-based and padded with -1.
The `cell_type` attribute keeps its VTK values (10/13/14); the actual MSH element types are
4/6/7. Elements are grouped by region and type, with their attributes reordered together.

Supported **input** VTU: a single UnstructuredGrid piece with four-node tetrahedra, ASCII or
uncompressed inline base64 arrays, UInt32/UInt64 binary headers and either byte order.
A missing byte_order defaults to LittleEndian for the existing paraviewo exports.
Compressed, appended, multi-piece and mixed-cell input files are rejected explicitly. The
hybrid result is intended for visualization/downstream use and cannot be loaded as a new
tetrahedral optimization input without first applying its stored decomposition.
Missing fields, invalid indices and invalid correspondence also cause an error.

Tests:

```sh
cmake --build build --target wmtk_test_prismatic_mesh -j 4
ctest --test-dir build -R '^wmtk_test_prismatic_mesh$' --output-on-failure
```
