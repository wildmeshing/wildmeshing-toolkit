# Prismatic mesh input

## Mixed shell and rod construction prototype

The opt-in `rod_mode` enables rod preparation inside a **shared shell/rod
optimizer**. Its construction may contain curves, sheets and input solids, with
explicit input labels and `corr_input_vid`. For example:

```json
{
  "application": "prismatic_mesh",
  "input": "rod_construction.vtu",
  "output": "rod_result.vtu",
  "rod_mode": true,
  "rod_radius": 0.04,
  "rod_axial_split": true,
  "iterations": 5,
  "prism_dominant": true,
  "keep_background_mesh": true,
  "export_input_and_shell": true
}
```

The VTU input needs `labels`, `vid`, `corr_input_vid`, `tag_0` (zero on a pure rod),
and `offset_tag`. MSH uses the existing `offset`/`ambient` physical groups and
explicit vertex labels/correspondence. Input volume is retained unchanged.
Rod edges are maximal input edges without input triangle/solid cofaces; ordinary
sheet and solid edges are not mistaken for rods. Rod attachment vertices shared
with faces/solids protect their complete band one-rings. `rod_radius` is an absolute length; zero uses `thicknessratio` times the
mean centerline edge length. `iterations` bounds preparation cleanup and the shared
collapse/smoothing iterations. Preparation aligns rod topology/frames without
relocating original vertices. Shell targets and distinct per-ring-vertex rod
targets then populate the same `target_positions` array. The shared collapse
queue retains rod ring vertices and prioritizes collapses towards them; the
shared single-vertex smoother reads each vertex target and checks its full star.
Shell and rod prism candidates are collected together for one reconstruction.

The centerline graph classifies degree-one endpoints and degree-greater-than-two
joints. Only non-rod attachments and branching joints hold their one-rings. A free
endpoint stops path traversal but receives an ordinary terminal frame and attempts
prisms up to that section. A rod with two attached ends keeps both connection stars;
a rod with one attached end protects only that end. Isolated closed rods have no
endpoint protection: an arbitrary traversal start supplies the same material frame
at both ends of the periodic frame sequence, and the closing interval is matched
and validated like every other interval. Degree-two vertices, including sharp corners,
participate in frame matching without an angle threshold; unsuccessful prism
conversions retain their valid tet subdivision.
Each rod stores a continuous directed edge path. Edge frames follow this path
and their cyclic material phases are aligned by quaternion transport. Every input
vertex frame is the equal-weight mean of its incident edge quaternions: endpoints
copy their edge pose, degree-two vertices use sign-aligned SLERP at 0.5, and
higher-valence joints use the principal-eigenvector quaternion mean. Section ring
windings follow the resulting vertex normal, including at right-angle corners.
Complete cyclic edge links define frame sectors; a frame's offset correspondence
must name one of that edge's endpoints. If a free terminal section has no edge link,
its cyclic rim is recovered from the lateral offset front's correspondence boundary.
The free end's complete circular material frame is defined in the normal plane through
the input endpoint. After reconstruction and optional axial splitting, every triangular
sector of that terminal disk must be the face of a real, positive-Jacobian prism.
Only after that check can the old offset construction's exterior cone be returned to
background. Thus the material ends on a full prism section; a historical pointed offset
cap does not remain a rod tet cap. Full-domain connectivity, positions, exterior boundary
and volume stay unchanged. Band-only output omits the exterior background cap.
The `terminal_frames` report records disk-sector checks and returned cap cells;
`rod_terminal_frame` and `rod_terminal_prism` expose the verified end in VTU/MSH fields.
Missing or incompatible sectors are explicitly reported and are never hidden by removing
the neighboring material. Existing non-rod attachments and branching joints retain their
protected connection stars.
Same-parent ring splits raise eligible
frame degrees to the maximum on each rod. These splits include all incident
background tets. If that target is blocked, a valid existing section remains available
for neighbors of the same degree; only incompatible intervals fall back. Missing regular
sections can be recovered from correspondence boundaries when neither adjacent edge link
owns the section. Redundant same-parent vertices may then collapse, preserving
joint one-rings. Frame vertex identities are provisional: the regular fiber
constructor can replace them during checked collapses, rebuild the actual edge
links, and split the resulting rings back to the common degree. Tau22 cleanup uses checked local split/collapse
operations; unresolved or incompatible links remain tetrahedral.

Circular section targets use these averaged vertex poses and their normal planes.
Only cyclic ring indexing is fitted; no independent vertex phase optimization
overrides the incident-edge mean. A single cyclic
matching between adjacent sections determines the product connectivity. Geometry moves
backtrack against **all supplied background cells**, even when the output omits
background. Blocked targets are reported rather than forced through the mesh.

Equal degree alone does not align an annulus's triangulation. Before hybrid
conversion, diagonal exchanges construct a consistent cyclic correspondence
between its two rings. Each exchange reconstructs its complete band/background
cavity with positive tets and the identical oriented cavity boundary. Background
kernel points support either diagonal; when possible, a band-side 2-3 exchange
returns the thin outer wedge to background instead. Frame planes are projected
only when every incident band and background tet stays positive. This stage
constructs the tau21/tau32 column partitions; the later three-tet matching check
validates the constructed columns. `frame_topology_construction`,
`annulus_diagonal_exchanges`, `planar_frames`, and `tau_after_frame_construction`
record its results, and the final frame report uses the updated ring vertices.

Constructed columns use tau21/tau32 reference tets. The existing prism Jacobian,
quad-interface and background-pyramid checks determine which columns can merge.
`rod_axial_split` inserts a midpoint input vertex and an equivalent circular frame
as one private construction, assigning new correspondence before building its two
halves. It reconstructs the neighboring background interfaces at the same time.
Only complete, compatible intervals are split; rejected proposals retain the
validated existing representation. There is no separate shell-edge presplit pass.

For an explicit comparison without background constraints, set
`"ignore_background_mesh": true`. This removes exterior tetrahedra before rod
preparation and omits background from optimization and hybrid reconstruction;
input volume and offset band remain. It overrides `keep_background_mesh`, and the
discarded background is not reattached. In the normal shared path,
`keep_background_mesh: false` only filters output and retains validity checks.

This is a conservative prototype: arbitrary tau22 configurations or incompatible
section triangulations are not guaranteed to normalize, and protected joint/attachment
ports and incompatible regions retain tets. Free caps do not force a tet one-ring.
Background-remeshing and Jacobian-smoothing options can
run in the shared optimizer; all background participates even when output omits it. Unmatched sectors retain their
valid tet subdivision, with conforming pyramid transitions where needed.

`<output_stem>_rod.json` records graph classification, frame degrees, blocked fits,
tau counts/parent checks, candidate counts and midpoint-frame results. These
diagnostics also include `rod_paths` with `edge_order`, `from_vid`, and `to_vid`,
endpoint kinds, ordered `frame_sequence`, shared periodic-frame status and closing
interval candidate counts. `free_endpoint_source_vids` distinguishes unprotected
terminal points; each section's `edge_index` also covers recovered terminal frames,
and each selected section exports its vertex normal and ordered `ring_vids`.
`edge_frames` and `vertex_frames` export the quaternions and incidence lists
needed to independently verify every original vertex mean. The memory
route returns the same diagnostics under `report["rod"]`; hybrid exports also
include them under `report["hybrid"]["rod"]`. Existing surface mode is the default.

Rod volume exports also carry cell fields for visualization: `joint_one_ring` is
1 on band tets incident to an actual joint vertex and 0 elsewhere. `rod_region`
separates background (0), prisms (1), other band tets (2), joint one-ring tets (3),
and other band cells such as transition pyramids (4). Color by `rod_region` in
ParaView to distinguish joint one-rings from the remaining tet transitions.

`construction_region` distinguishes background/input volume (0), shell band (1),
rod band (2), and mixed rod/shell transitions (3). `rod_region` retains its original
cell-kind coloring for compatibility.

Mixed-input fixtures are under `mydata/mixed_shell_rod/20261109` (connected
solid/sheet/rod attachments, conforming stellar offset construction) and its
`native_components` subdirectory (disjoint native curve and solid/sheet offset
components welded across an unchanged background grid plane before one optimizer).
The upstream `topological_offset` currently refuses triangles together with free
segments in one distance-query complex. These fixtures test a supplied mixed
construction; they do not change that upstream restriction.

### Fixed-frame corner buffer experiment

Set `rod_corner_preview_vid` to a degree-two input source ID to export an isolated
curve-only corner patch instead of the full mesh. `rod_buffer_max_angle` (default 15 degrees)
sets the initial SLERP sampling step. `rod_buffer_max_subdivision` defaults to 3:
after a failed geometric construction, both side layer counts double, up to three
additional binary refinement levels. Each side also has a 128-layer budget.
The corner position and full local pose are fixed;
the two outer interfaces keep their existing coordinates and source IDs. New
input centers lie on the original two segments, with correspondence assigned to
their rings during construction. The reference mesh has three positive tets per
prism and is checked with the normal hybrid validator.

The fixed corner pose is always the mean of the two incident edge frames,
including when no eligible offset ring was selected at that vertex. It is not
recovered from an individual ring or offset plane. The complete quaternion,
including material twist, is retained. Outer cyclic index origins may be shifted
to choose the closest equivalent indexing before interpolation.

This preview constructs actual local volume cells; it does **not** replace the
old background cavity or merge the patch into the full mesh. Diagnostics state
`background_integrated: false`. Source mesh and ordinary rod-mode output remain
unchanged. A valid prism construction is accepted as soon as its Jacobians,
reference tet split and hybrid interfaces pass. Both compatible input-apex
reference splits are tried. When the subdivision budget is exhausted, the
preview returns the unchanged local reference tets with `status: tet_fallback`.
`subdivision_attempts` records every tested depth and its failure reason.
Invalid arguments still report errors. The turn angle does not exclude a corner.

For the crossing-quads trial:

```sh
./build/app/wmtk_app -j mydata/rod_cross_quads/20261105/prism/diagnostics/corner_buffers/config_1154.json
```

The crossing-quads experiment also has a full integration driver:

```sh
/Users/cavers/miniconda3/bin/python experiments/rod_cross_quads/integrate_corner_buffers.py
```

It reruns the native rod pipeline and attempts every turn in the curve (or the
explicit `--corners` list), then replaces successful local band/background cavities.
Each new shell quad receives a positive background pyramid. TetGen fills the
remaining background with boundary splitting disabled (`-pYQz`); the driver
requires the output boundary triangles to equal the input triangles exactly.
It also checks the replacement cavity's oriented boundary and volume with exact
rational arithmetic, retains all cells outside the cavities, and runs the full
mesh audit. This is an experiment driver; ordinary `rod_mode` does not invoke an
external background tetrahedralizer automatically.
Failed requests leave their original global regions intact; background integration
also uses a private snapshot and rolls back on failure. Use `--max-subdivision 3`
to set the refinement budget. `corner_buffer_attempts.json` records successful
buffers and all tet fallbacks; `corner_buffer_integration.json` contains only
the patches actually integrated into the full mesh.

The driver expects `build/rod_corner_tools/tetgen` or `--tetgen <executable>`.
This experiment used [libigl/tetgen](https://github.com/libigl/tetgen) revision
`e05aca7df74e3f531bc35733ed87d36d437266c5`, compiled as a standalone tool. The
downloaded source and license are cached under `build/rod_corner_tools/src`.
To reproduce the quads, offsets, native prisms and integrated corners together,
run the crossing-quads generator with `--corner-buffer` (implies `--prism`).

The corrected incident-edge-average results are under
`mydata/rod_cross_quads/20261105/prism/corner_vertex_averaged`:
`result.vtu` includes background; `result_input_shell.vtu` contains the rod.
All eight corners produce valid prism buffers and integrate with background.
The full mesh contains 816 prisms, 816 background pyramids and 404 band tets
(including 124 protected joint one-ring tets). The independent audit verifies all
69 original vertex quaternion means, exact positive reference tets, conforming
hybrid interfaces and exact domain volume 27. `corner_418_averaged_frame.png`
shows the corrected vertex pose and actual buffer cells. The earlier
`corner_buffered_bounded` snapshot predates the vertex-frame correction.
`corner_buffer` marks the added corner prisms and reconstructed background by
their original corner ID. `preview.png` colors the new prisms blue, while
`corner_fallback` marks the original tets retained after failed corner attempts.
`corner_subdivision_depth` records the tested/accepted depth on these cells.
`background_section.png` shows the actual reference mesh through corner 1154.

Native mixed-element AMIPS is available with:

```sh
/Users/cavers/miniconda3/bin/python experiments/rod_cross_quads/analyze_element_amips.py
```

It evaluates each actual four-node tet, six-node prism or five-node pyramid using
`F = J_physical * inverse(J_ideal)` and `AMIPS = ||F||^2 / det(F)^(2/3)`.
The ideal prism has an equilateral unit triangle and unit height; the ideal
square pyramid has eight equal unit edges and height `1/sqrt(2)`. All ideal
elements score 3. `amips_element` is the maximum over the element. Prism maxima
use the triangle-vertex paths and the stationary roots in height; pyramid maxima
use the four collapsed-square corner paths, analytically canceling the ideal
map's apex collapse. The evaluator checks ideal elements, similarity invariance,
inverted-element rejection and shape-function Jacobians by finite differences.

`result_element_amips.vtu` preserves the complete mixed connectivity, and
`element_amips_quality.json` records the references, method and statistics.
`analyze_amips.py` provides a separate diagnostic of the stored reference tet
decomposition; those values are not the native prism/pyramid element scores.

## Surface construction

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
  "min_tet_volume": 0,
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
`prism_dominant.cpp`; existing vertex positions and attributes remain intact. Feasible
background interface cavities gain a new kernel point and a pyramid. The band tetrahedral
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

Before extracting the band, `handle_open_boundaries(input)` processes the full construction
mesh according to Section 4.2.6. An open input edge has exactly one incident shell input
triangle and belongs to no input-volume tetrahedron (`input_cells == 1`). A shell input
triangle has three input vertices and an incident band tet; shared triangles are counted once,
including on two-sided sheets. Input-volume membership is checked before filtering the mesh.
Candidates are band tets in the union of the two input endpoint stars, so they include
configurations with only one input vertex.

Protection uses curve endpoints, not an interior-fan classification. Count incident
input surface triangles per edge: one identifies an input open-boundary edge; more than
two identifies a shell-intersection edge. A shell-intersection graph vertex with degree
one is an endpoint. Protect its one-ring only if it is not an input open-boundary vertex.
Interior vertices along X or T intersection lines are not protected merely by their
nonmanifold incidence. Rod ends attached to non-rod faces/solids and branching rod
joints protect their connection stars, including attachments on shell open boundaries.
Free rod ends and isolated loops have no protected one-ring. Path stops are classified
separately from protection, so free ends still terminate graph traversal safely.
The open-boundary exclusion applies only to shell-curve endpoints.

`initialize_junction_protection()` captures each connection point's band-tet one-ring
before rod preprocessing or open-boundary trimming. Collapse cannot remove an offset
vertex belonging to those original cells; unlock and rod preprocessing cannot split or
collapse a cell containing a protected center. A check after shared optimization verifies
that every original protected tet's connectivity survives. This is a no-collapse rule,
not a blanket freeze of all nonmanifold vertices or of all vertices with the same parent.
The protected offset vertices retain their distinct original positions instead of following
a shared component target that assumes they can collapse. Target smoothing and optional
Jacobian smoothing skip these vertices; ordinary vertices continue to optimize. Existing
rod-attachment/joint constraints still apply separately.
The hybrid report's `junction_protection` records the two kinds of centers, original
protected-cell count, no-collapse offset vertices and connectivity verification result.
Output fields `connection_kind` (1 protected rod attachment/joint, 2 off-boundary shell-curve end, 3 both),
`no_collapse` and `connection_one_ring` make the protected region inspectable.

Set `"transition_smoothing_iterations": 8` for optional geometry relaxation after hybrid
reconstruction (default 0, range 0--100; requires `prism_dominant`). The no-collapse mask
still protects topology, while protected and marked outer offset vertices and one adjacent
surface ring may move. Input points and completed rod-prism frames remain fixed. First,
inward correspondence rays relax toward the convex envelope of the input vertex's active
one-ring, computed from an immutable snapshot. Then a bounded local descent reduces squared
positive signed heights across oriented surface hinges. This penalizes concave folds without
flattening already convex parts. It is a local objective, not a guarantee of global convexity
when the fixed input or interface requires concavity.

Each move preserves exact-positive reference tets, the volume floor for modified band tets,
both pyramid decompositions and analytic global prism Jacobians. Reference tet determinants
and minimum native Jacobians retain at least 25% of their pre-smoothing values.
Native tet/pyramid AMIPS cannot exceed the greater of 10
and its initial value; prisms use a conservative native AMIPS bound from the maximum corner
Frobenius norm and the global minimum determinant. Total motion is limited to half a local
mean surface edge length; fairing retains at least half the initial column's projection along
its original direction. Failed moves backtrack or leave the point unchanged.

The `transition_smoothing` report records before/after coordinates, concavity energy and
the safeguards. Reconstruction-only position/boundary checks are distinguished from final
geometry, since this stage intentionally adjusts the offset boundary. The no-collapse and
connection fields continue to describe topology protection even when those coordinates move.

Stacked offset constructions may use the same vertex as one layer's offset and another
layer's source. The readers retain the raw source/target links in
`junction_protection.layered_correspondence`, compose chains to their original input root,
and protect the shared vertices, affected descendants and source-volume offset vertices as
junction stars. Cyclic or unresolved chains remain errors. Source curve/sheet simplices
provided by the memory input are retained; a shared vertex keeps its original geometry
without being relabeled as an ordinary input vertex. Existing volume vertices also remain
fixed even when they carry offset correspondence. Protected junction cofaces with composed
parents are counted separately from invalid ordinary rod cofaces.

This is a conservative union-band policy, not a separate role/optimizer per offset layer.
Large or extended overlaps can remain mostly tetrahedral. `connection_kind` uses bit 4 for
these layered junction centers, alongside bits 1 (rod connection) and 2 (shell terminal).

The type `tau^n_m` counts offset vertices (`n`) and their distinct input identifiers (`m`).
The pass unmarks `tau^2_1` only when the edge between its two input vertices is on the open
boundary, and unmarks `tau^3_1`. It unmarks `tau^3_2` unless its distinct labels map to an
existing input edge outside the open boundary. `tau^1_1` and `tau^3_3` are preserved.
A `tau^2_2` is unmarked only when its offset label set matches its two input vertices and
that input edge is open. Decisions use the initial boundary and candidate sets, with resolved
correspondence rows preserving the meaning of noncontiguous source IDs.

Unmarking only sets `offset_tet_tags` to -1. The pass preserves all tetrahedra, coordinates,
input-volume tags and vertex attributes. The subsequent extraction computes targets on the
remaining band; with `keep_background_mesh=true`, unmarked cells join the background
optimization domain. They are excluded from the shell unlocking schedule. Logs report open
edges, candidate tets, unmarked tets, junction vertices and protected candidate tets.

`evaluate_target_positions(input, thicknessratio)` then computes targets without moving
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
For a positive iteration count, a final exhaustive collapse pass runs after the
last smoothing sweep. This consumes edges that became admissible at the final
positions before hybrid marking; otherwise they would seed unnecessary
pyramid/tet transitions. It uses the same correspondence, protection, link and
volume checks, and its counts are included in the last optimization record.
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
(default 0, finite and nonnegative). Zero requires strict positivity without an
absolute size cutoff. An explicitly positive floor has coordinate units cubed;
it is not a normalized shape-quality metric.
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

Offset vertices carry a binary transition mark. On a repeated-correspondence boundary face,
only the two endpoints of each same-correspondence edge are marked; the other corner of an
`(a,a,b)` triangle is not a collapse seed. Only the protected endpoints described above mark their complete one-ring columns
as tet regions. A regular nonmanifold crossing is not automatically marked or locked.
An unmatched/fixed triangular interface without an accepted background pyramid marks its
offset edge endpoints at initialization.

Each offset triangle determines its column's output shape: zero marked corners produces a
prism; one produces an input-apex pyramid plus a tet; two or three produces a tet region.
The unique marked corner fixes the pyramid direction: for marked `a'` on input/offset
triangles `abc` / `a'b'c'`, the pyramid has base `bcc'b'`, apex `a`, and the remainder is
`aa'b'c'`. A conforming direct triangular interface between a shell pyramid and an unmatched
band tet is valid; no extra prism-derived tet buffer is required. The input triangular caps
and their adjacent input-volume tetrahedra stay unchanged.

When background is retained, each candidate lateral quad may instead receive a background
pyramid. Its two reference triangles must each have one band and one background owner.
The cavity is the connected background sector of the existing quad-diagonal edge star;
its link must be a simple path between the other two quad vertices and its boundary a
closed triangulated sphere. Input-volume cells and mixed background region tags are excluded.
Cavities are reserved deterministically and cannot overlap. Only reservations whose quad
survives the shell shape solve are applied.

With cavity boundary vertices fixed, a normalized four-variable linear program maximizes
the new apex's minimum signed distance to all cavity boundary planes and the four pyramid
corner planes (both base diagonals). The implementation enumerates rank-four active sets;
at most 64 constraints are admitted per cavity to bound work. A positive clearance is followed
by exact rational positivity/floor tests on every coned tet and all four pyramid corner
determinants, plus exact cavity volume conservation. The two cones at the quad base merge
into the background pyramid; all other boundary cones remain background tets. New points
have unique source IDs, no input correspondence, and inactive target/component attributes.
Existing points, cavity boundary triangles, input cells and non-cavity background cells stay
unchanged. A geometric failure, overlap or search limit leaves the original interface for
ordinary conforming prism/pyramid/tet degradation; it never emits an unmatched quad.
The search-limit reason is distinct from an empty positive kernel. There is no vertex-moving
nonlinear repair or enlargement of an infeasible cavity in this pass.

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
A failed prism Jacobian check marks one previously unmarked offset corner on the column
edge with the lowest Jacobian. A failed pyramid check marks the offset corner associated
with its worst base-corner Jacobian; all four normalized pyramid corner determinants are
checked by requiring both tetrahedral decompositions to be positive. Geometry failures,
volume-floor failures and a missing positive pyramid/remainder split have distinct reasons.
Marks are monotone and affected shapes are reconstructed until the prism and pyramid
checks pass. Existing small positive band tets may remain or be grouped into a valid prism
without turning the operation floor into a program failure.

Prisms and matched tet regions can select any positive three-tet decomposition to satisfy
pyramid side orientations. Tet regions participate in the same solve rather than retaining
an incompatible old diagonal. Each shared quadrilateral carries one consistent reference
diagonal, including quad/quad interfaces. Interfaces to fixed/unmatched cells retain their
triangulations. An exterior quad can change diagonal only when exact predicates certify
that it is planar and strictly convex. Validation checks its two-triangle coverage and
oriented four-edge boundary, so this changes the triangulation without changing the domain
geometry; nonplanar exterior patches remain fixed.

Constraint propagation followed by deterministic search selects a positive joint assignment,
preferentially retaining original splits. If these fixed interfaces admit no positive joint
assignment, one additional offset corner of the affected one-mark transition is marked and
reported as `no_compatible_positive_transition`, separately from a Jacobian failure. This
is a remaining conservative fallback for fixed/nonplanar interfaces, not a proof of an
invalid pyramid Jacobian. Search uses at most 10,000 states per attempt and records a search
limit separately. No failed solve freezes a whole column or automatically marks all three
corners. Every unsuccessful iteration adds a new mark, so the marking loop terminates.
Unmatched tet cores themselves are not remeshed in this pass.

The final reference tetrahedral mesh is built privately and committed only after validation.
Existing vertex coordinates, IDs, correspondence, normals and targets, input cells and
non-cavity background cells remain unchanged. The output point field `transition_mark` is
0/1 on offset vertices and -1 on other vertices. The hybrid report includes
`marker_iterations`, per-corner `marker_updates` with reasons, `marked_offset_source_vids`,
`flexible_planar_exterior_quads`, and `interface_reconstruction_updates`.

The hybrid JSON report records the Jacobian sample positions, minimum sampled determinant
among output prisms, `invalid_prism_jacobian` rejections, and the histogram
`jacobian_valid_candidate_admissible_split_counts`. Validation reports sampled and global prism Jacobians,
positive reference tets, new-tet volume floors and pyramid decompositions separately.

Validation checks positive reference tets, exact coverage of the final reference mesh,
triangle/quad interface agreement, opposite face orientations, input-side shell pyramid apices
and background kernel-point pyramid apices.
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
| `background_transition` | 1 for a reconstructed background pyramid or one of its supporting cavity tets |

The decomposition field preserves the precise prism/pyramid subdivision and the shared-face
diagonals for downstream tetrahedralization. `<output_stem>_hybrid.json` records cell counts,
validation results, rejected candidates and transition/tet regions. Its `retriangulated_regions`
records the original and new tets using source vertex IDs, along with the affected reference row
IDs. After a split changes, `source_tet_ids` refers to these final reference rows, not a claim
that the new cell is a union of those pre-reconstruction tets. In ParaView, color the
volume by `cell_type`; select `offset_tag = 1` to inspect only the reconstructed band.
`shell_pyramids` and `background_pyramids` split the total `pyramids` count in the report.
`background_pyramid_reconstruction` records accepted/applied cavity counts, rejection reasons,
new point rows/IDs, original and replacement reference rows, clearance and exact checks.
Unchanged reference row IDs are retained; removed cavity rows are reused before appending
additional reference tets. Input/shell-only exports omit the new background points and cells.
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
