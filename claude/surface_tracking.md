# Surface tracking: recording, mapping, extending

Notes on the tracking framework added for BijectiveRemesh (arXiv 2605.30744): how it works, how
to use it, how to bring it to another application, and how to check that it is right. Written
from the implementation on the `danielepanozzo/surface-history` branch; file paths are relative
to the repository root.

## What it is for

An application that remeshes a surface -- or a tet mesh whose tracked faces are a surface --
can record every change it makes to that surface. From the record (a *history*, saved as a
*tracking file*) the `surface_map` component rebuilds a map between the input surface and the
output surface, both ways. Users map their own points, and through them their own attributes:
labels, texture coordinates, any per-vertex or per-face data.

The map is built as BijectiveRemesh builds it: one local map per operation, between the
patch the operation replaced and the patch it left, composed in order. Where every local map
is a bijection, so is the whole map.

## Quick start

Turn tracking on with `history_output` -- the same option in every application that has it:
`isotropic_remeshing`, `qslim`, `shortest_edge_collapse`, `tetwild`, `simwild`.

```json
{"application": "tetwild", "input": ["model.obj"], "output": "out", "history_output": "out.wmtkhist"}
```

On the memory route (`name(options, meshes)`, see `docs/in-memory-io.md`) no file is written:
any non-empty `history_output` records the history, and the report returns it as `history`, the
bytes of the file the file route writes (bytes in Python). `history::write_history` /
`wm.write_history` save them as that file. simwild also returns the mesh its history starts from
as the output mesh `history_input`.

Map points with the `surface_map` application of the same `wmtk_app` binary:

```json
{"application": "surface_map", "history": "out.wmtkhist", "operation": "map",
 "points": "points.txt", "output_points": "mapped.txt", "direction": "to_output"}
```

`points.txt` holds one point per line; numbers separated by blanks; `#` starts a comment.
- On the input: `face b0 b1 b2`. `face` is a triangle of the input files, counting from 0 over
  all of them in order, as the files hold them. `b` are barycentric coordinates with respect to
  that triangle's corners, in file order.
- On the input of a history that starts at a tet mesh's tracked surface (`simwild` on a
  `.msh`): `v0 v1 v2 b0 b1 b2`, as on a tet output, with the `v` vertices of the tet mesh the
  history starts from (`<output>_history_input.msh`).
- On the output of a surface application (`isotropic_remeshing`): `face b0 b1 b2`, with `face`
  a triangle of the output file.
- On the output of a tet application (`tetwild`, `simwild`): `v0 v1 v2 b0 b1 b2`. The `v` are
  three vertices of the output tet mesh, at the corners of a tracked face. The tracked surface
  is no element of the tet mesh file, so the vertices name the face. They may come in any order;
  the coordinates follow them.

`direction` is `to_output` or `to_input` (the inverse map). A point with no image comes back
with id `-1`.

From C++ (`components/surface_map/wmtk/components/surface_map/Tracking.hpp`):

```cpp
const auto tracking = surface_map::Tracking::load("out.wmtkhist");
std::vector<surface_map::InputPoint> pts = {{face, {b0, b1, b2}}};
auto on_output = tracking.to_output(pts);    // OutputPoint: .face, or .vertices for a tet output
auto back = tracking.to_input(on_output);    // InputPoint: .face, or .vertices (input_by_vertices)
auto triangles = tracking.output_triangles(); // the output's tracked faces, in its own ids
```

Worked examples, one per application, are unit tests:
- `components/surface_map/wmtk/components/surface_map/tests/test_tracking.cpp` (isotropic)
- `components/tetwild/wmtk/components/tetwild/tests/test_tetwild_tracking.cpp`
- `components/simwild/wmtk/components/simwild/tests/test_simwild_tracking.cpp`
- `components/qslim/wmtk/components/qslim/tests/test_qslim_tracking.cpp`
- `components/shortest_edge_collapse/wmtk/components/shortest_edge_collapse/tests/test_sec_tracking.cpp`

Each one runs the application with tracking on, maps random input points to the output and
back, remaps a per-vertex attribute onto the output's vertices, and does the same from the
command line with a points file.

Remapping attributes:
- **Per face.** Map each output face's centroid `to_input` and take the input face's value.
  The `transfer` operation does this.
- **Per vertex.** Map each output vertex `to_input` as a corner of one of its faces
  (barycentric `(1,0,0)` and its permutations) and interpolate there.
- **Per corner, with discontinuities** (texture coordinates and their seams). The `transfer`
  operation evaluates each corner just inside its face, so that a corner on a seam lands in
  its own chart. This needs the output to conform to the seams (see *Features* below).

## The history

`src/wmtk/history/SurfaceHistory.hpp`. A `SurfaceHistory` holds:
- **A snapshot** of a triangle complex: vertex and face *slots*, live or not.
- **Feature edges:** curves the edits keep. See *Features* below.
- **The edits,** in commit order.
- **`metadata`:** free text. The applications store their JSON parameters there, which is how
  `surface_map` finds an application's input files when it is not given them.
- **`output_vertex_ids`:** for applications whose output numbers vertices its own way.
- **`input_vertex_ids`:** the same for the first state, for a history that starts at a tet
  mesh's tracked surface rather than at input files: the mesh's id of each snapshot vertex
  (`TrackedSurfaceRecorder` fills it). Empty for a history from files.

Edits are described on the complex alone -- vertex and face ids, positions -- never on the
mesh class that did them. A TriMesh and the tracked surface of a TetMesh record in the same
terms.

### Edit kinds

| kind | ids | meaning |
|---|---|---|
| `split_edge` | `v = {v0, v1, new}`; faces in pairs `(f, new_f)` | each face on the edge keeps its `v0` half (`v1` renamed `new` in place); `new_f` is `f` with `v0` renamed `new` |
| `split_face` | `v = {v0, v1, v2, new}`; faces `{f, f1, f2}` | `f`, `f1` and `f2` are the three faces at the new vertex, each the old face with one corner renamed |
| `collapse_edge` | `v = {removed, kept}` | faces with both go; `removed` is renamed `kept` elsewhere; duplicate faces merge into the smallest id; a vertex left without faces is retired |
| `swap_edge` | `v = {v0, v1, a, b}`; faces `{fa, fb}` | edge `(v0,v1)` becomes `(a,b)`: in `fa` (apex `b`) `v1` becomes `a`, in `fb` (apex `a`) `v0` becomes `b` |
| `move_vertices` | none | only positions change |
| `consolidate` | none | live vertices and faces renumbered by rank |
| `replace_faces` | `v` = corners of added faces; faces = removed, then added | a change no other kind describes; no correspondence; the map cannot follow it |
| `relabel` | `v` = new id per vertex slot; faces = new id per face slot, or empty for rank order | vertices renamed in place (several may become one); faces with a repeated corner go. The weld, a coordinate change, a reordering. A point keeps its barycentric coordinates |
| `embed` | `v` = corners of the new complex; faces = per new face, its sources (count, then ids), then feature edges | the complex is replaced by one lying on it (an insertion). A point keeps its place in space |

Every edit carries `moves`: the position, after the edit, of every live vertex it wrote.
Positions that did not change are dropped.

`Replay` (`src/wmtk/history/Replay.hpp`) applies edits with exactly these semantics. After the
k-th edit it holds the recording mesh's surface after its k-th committed operation: the same
ids, corner order and positions. Everything downstream relies on that.

Composition (`SurfaceHistory.hpp`):
- **`history::append(h, next)`.** `next` starts exactly where `h` ends. It checks live
  vertices and faces, then appends `next`'s edits.
- **`history::append_embedded(h, next, sources)`.** `next` starts on a complex lying on `h`'s
  last state. It appends an `embed` edit, then `next`'s edits.

### File format

Binary, varints (`SurfaceHistory.cpp`). Version 7 is the first released one; the earlier ones
were development formats and no longer load. After the version comes one byte, the number of
corners of a face: 3, a surface. When you add to it, bump `kVersion`, keep loading version 7
and up, and note in the comment at the top of the file what each version added.

In memory a history is the bytes of its file: `to_bytes()` and `SurfaceHistory::from_bytes`,
which the memory route of a component returns and takes. `history::read_history(path)` and
`history::write_history(path, bytes)` move those bytes to and from a file unchanged, as
`io::read_mesh` and `io::write_mesh` do for a mesh; they check only the header. `save` and
`load` are the two together.

Loading checks what it reads before using it, so a damaged file throws "is corrupt" instead of
crashing:
- every count is bounded by the bytes left (each item takes at least one);
- the snapshot's faces name live vertices, and its feature edges existing ones;
- each edit's ids name live vertices and faces of the replayed state (`ids_valid`), and the
  ids it creates stay within what the rest of the file could still announce;
- an `embed` has a move for every vertex it brings, corners and feature edge ends alike.

A new edit kind needs its case in `ids_valid`. Tests: `history-load-refuses-damaged-files`,
`history-load-refuses-what-a-map-cannot-follow`, `surface-map-damaged-history`.

### Features

`feature_edges` are curves the map must take onto themselves: label boundaries, UV seams,
tetwild feature curves. They are listed only in the snapshot (or in an `embed`). The edits
carry them by fixed rules, which the recording mesh must follow:
- **Split:** a split feature edge leaves two feature halves.
- **Collapse:** the removed vertex's feature edges go to the survivor; the collapsed edge dies.
- **Swap:** never takes a feature edge.

The map flattens each side of a feature curve separately (`sides_of` in `SurfaceMap.cpp`). A
non-manifold edge -- three or more faces -- separates sides without being declared.

## Recording a TriMesh (surface only)

`src/wmtk/TriMesh.{h,cpp}`. `start_history(position)` snapshots the mesh and returns a
`history::Recorder`. From then on every operation commits an edit at its commit point:
- `split_edge`, `split_face`, `collapse_edge`, `swap_edge`, `smooth_vertex`, and
  `consolidate_mesh`.
- The commit point is just before `release_protect_attributes()`, while the operation still
  holds its locks. Concurrent commits therefore append in a valid serial order (the recorder
  has a mutex).
- The vertices written come from the attribute journal (`append_recorded`); their positions
  come from `position(v)`. It is called inside operations, so it must read through const
  access.

Rules for an application on a TriMesh:
- **Change the surface only through these operations.** Anything else that changes
  connectivity or positions escapes the history.
- **Feature curves:** follow the rules above, and put them in the snapshot's `feature_edges`.
  `isotropic_remeshing` does this from `feature_fields`.
- **Number the output file by the history's last state.** Consolidate, then write faces in id
  order with their stored corner order. Then output face ids *are* the history's.

Users: `isotropic_remeshing` (`history_output`, `feature_fields`), `qslim` and
`shortest_edge_collapse` (`history_output`), and the input simplification of tetwild and simwild
(`ShortestEdgeCollapse`).

## Recording a tracked surface on a TetMesh

A tet mesh's tracked surface is a subset of its faces, and faces have no stable ids: a face's
attribute slot is `4 * lowest incident tet + local index`, which moves whenever its tets
change. So the tracked surface gets ids of its own, in
`history::TrackedSurfaceRecorder` (`src/wmtk/history/TrackedSurface.{hpp,cpp}`).

- **Its own ids.** The recorder keeps a surface vertex id per mesh vertex on the surface, and
  surface face ids through its own `Replay`. A mesh renumbering (`consolidate_mesh`) only
  renumbers that table, with no edit.
- **Commits as diffs.** At each commit the mesh reports the tracked faces around the
  operation, before and after, as sorted vertex triples. The recorder takes the difference and
  finds the edit that explains it:
  - **Split:** every face on the edge goes, and its two halves come.
  - **Collapse:** the faces at `removed` are renamed `kept`, faces with both go, and duplicates
    merge. If `kept` was not on the surface, it takes the removed vertex's surface id and the
    edit is a move. If both ends were on the surface and no tracked face held the edge, it is a
    *weld*, which is counted.
  - **Swap:** two faces `(a,b,c)` and `(a,b,d)` become `(a,c,d)` and `(b,c,d)`.
  - **Nothing changed:** a move, if a surface vertex moved.
- **Verified before recorded.** The predicted result must equal the actual faces after.
  Otherwise the change is recorded as it is (`replace_faces`) and counted in `stats().replaced`.
  `stats().unknown` counts faces the recorder had lost track of; it should always be 0.
- **Orientation.** A face keeps the corner order the edits give it. A face only a
  `replace_faces` makes takes `orient_tracked_face`.
- **`resync(faces)`.** Brings the surface in line with the mesh's tracked faces, after a change
  outside the operations, as one `replace_faces`.

`TetMesh` (`src/wmtk/TetMesh.{h,cpp}`) calls the recorder:
- **Start and stop:** `start_surface_history(position)` and `stop_surface_history()`.
- **Capture.** Each operation captures the tracked faces around it once its before-hook and
  screening have passed (`surface_history_before`):
  - split: faces touching `{v1, v2}`;
  - collapse: faces touching `{removed, kept}`;
  - swaps: faces with all three corners among the ring vertices.
- **Commit.** It reports at its commit point (`surface_history_commit`). Hooked:
  `split_edge`, `collapse_edge`, the 3-2 `swap_edge`, the 2-3 `swap_face`, `swap_edge_44`,
  `swap_edge_56`, and `smooth_vertex`.
- **Renumbering and resyncing.** `consolidate_mesh` renumbers the recorder.
  `remove_tets_by_ids` (tetwild's filters) resyncs it.
- **Virtuals for the application:**
  - `is_tracked_face(tid, j)`: which faces are tracked;
  - `orient_tracked_face(f, tets)`: their corner order, given the face's tets. The
    one-argument form looks the tets up; callers that hold only one corner pass them from
    `faces_at_vertex` (see *Locking*);
  - `tracked_feature_edges()`: the curves on the surface.
- **Locking.** An operation holds its seed's one-ring (`{v1, v2} ∪ N(v1) ∪ N(v2)` for an
  edge), and only part of the two-ring: see the note above `try_set_vertex_mutex_two_ring`.
  - Reading the star of a held vertex u is race-free: anything that changes a tet at u must
    hold u.
  - The corners across u's star are two rings out and may not be held. Another thread can be
    rewriting *their* tet lists. So find the star's faces from u's tets alone
    (`TetMesh::faces_at_vertex`: face ids and tets included). Do not use `tuple_from_face`,
    `Tuple::fid` or `tets_of_face`, which read all three corners' lists. The fold check first
    did, and segfaulted now and then at 8 threads.
  - Their positions are safe to read: moving a neighbour of u needs u.
  - Dry runs (screening) return before any hook.

`TetOptimizerMesh` (`src/wmtk/TetOptimizerMesh.{h,cpp}`), the base of tetwild and simwild,
fills these in:
- **`is_tracked_face`:** `m_is_surface_fs`.
- **`orient_tracked_face`:** the input orientation `m_orientation` on the face (its id from
  its lowest tet), ascending vertex order where it is 0.
- **`tracked_feature_edges`:** tagged feature edges (`m_feature_edge_attribute`) that are edges
  of tracked faces. tetwild's operations keep them by the history's rules: a split halves one,
  a collapse remaps, swaps refuse them.
- **`record_surface_history(metadata)` / `finish_surface_history(input)`.**
  - `finish` resyncs, warning if anything drifted.
  - It checks that the history's feature edges equal the mesh's, warning if not.
  - It fills `output_vertex_ids`, logs the counts, and returns the history, which the
    application saves with `ComponentIO::save_history`.
  - With `input`, it returns the chain from the input files; see the next section.
- **`update_attributes_and_history()`:** `update_attributes()` followed by a resync, so a
  surface re-derived between passes is recorded.

`SimWildMesh` overrides `orient_tracked_face`. Each face points out of the tet whose tag set
comes first, so a sheet between two tag sets is oriented the same way everywhere, from the
tags alone.

## From the input files: the chain

The tet histories, and isotropic's, start at the input files as read, so that a user's
triangle id is a triangle of their own file. The stages before the mesh go into one history:
- **Read and weld.** `io::read_triangle_mesh(paths, ..., ReadProvenance*)` returns the
  meshes as read, concatenated, and the vertex each became. The snapshot is the raw meshes,
  followed by a `relabel` to the welded mesh. isotropic maps its feature edges onto raw
  vertices, and the relabel carries them over.
- **Simplify.** The simplifier is a TriMesh. Record it with `start_history` from the welded
  mesh, then `history::append` (tetwild, simwild's `EmbedSurface::simplify_surface`). Its
  consolidation makes the ids the arrangement's triangle ids.
- **Insert.** `utils::embed_triangles_in_tets` can return, per facet, the input triangles it
  lies on (`EmbedTrianglesProvenance::want_face_triangles`).
  - **Exact sources.** A facet's sources are the triangles of its coplanar group that it
    overlaps with positive area: a separating-axis test in the group's plane, in doubles where
    a margin decides, in rationals otherwise.
  - **Pitfall: coplanar groups are not cut along their internal edges.** A UV sphere's quads
    are two coplanar triangles, and a facet can lie across both. The centroid test alone
    missed this.
  - **Matching facets to tracked faces.** The applications key each facet's sources by its
    corners' positions, sorted (`TetOptimizerMesh::PositionKey`). The tet mesh's `m_posf` is
    the rounding of the same rationals, bit for bit, so `record_surface_history` finds every
    snapshot face's sources there (`m_insertion_sources`), and warns about any it does not.
  - **The saved chain.** `finish_surface_history(&input_history)` appends an `embed` onto
    the tet history's first state, then the tet edits.
- **simwild's surface route.** `EmbedSurface` records per-file read and weld, the transform,
  the split into components and the concatenation as one `relabel` with an explicit face map,
  then the optional simplification, then the embedding. simwild also writes the tet mesh the
  history starts from as `<output>_history_input.msh`.

| application | history starts at | ends at | output point |
|---|---|---|---|
| `isotropic_remeshing`, `qslim`, `shortest_edge_collapse` | the input file as read | the output file | output triangle + barycentrics |
| `tetwild` | the input files as read | the tracked faces of `<output>_final.msh` (after any filter) | three tet vertex ids + barycentrics |
| `simwild`, `.msh` input | the tag interfaces of `<output>_history_input.msh` (input points: three of its vertex ids + barycentrics) | the tag interfaces of `<output>.msh` | three tet vertex ids + barycentrics |
| `simwild`, surface input | the input files as read | the tag interfaces of `<output>.msh` | three tet vertex ids + barycentrics |

## The map: `surface_map`

`components/surface_map/wmtk/components/surface_map/`:
- **`SurfaceMap`.** Built from a history alone. It replays the edits and keeps a *step* per
  edit:
  - **Splits:** exact affine maps of the new faces onto the old.
  - **Collapses and swaps:** a joint flattening of the patch before and after on a shared
    outline (Liu et al. 2021).
    - Tutte embeddings on the unit circle start it, the outline spaced by 3D length.
    - SLIM (`slim2d.cpp`, Rabinovich et al. 2017) lowers the distortion of both patches at
      once. They share the outline's vertices as unknowns, so the boundaries match by
      construction. Its line search keeps every triangle positive by exact `orient2d`.
    - A scaffold (Jiang et al. 2017) keeps the outline a simple polygon (`Scaffold`):
      - `box`, the default, as in the paper. The region between the outline and a box three
        times the patch, triangulated anew before every SLIM iteration, each triangle resting
        in its current shape. `box_scaffold` (`scaffold2d.cpp`) is a constrained Delaunay
        triangulation from VolumeRemesher's 2D arrangement (`vr2d::build_arrangement`: Delaunay,
        then the outline's edges forced in by flips). Nothing crosses, so it adds only corners;
        the outside is a flood fill from them that never crosses the outline.
      - `ring`: one band of triangles out to a fixed copy of the outline, resting in its first
        shape. It pulls the outline back and caps how far it stretches; renders of local maps
        made that plain (a long thin patch: distortion 44 with the ring, 4.0 with the box).
      - Measured 2026-10-22, box against ring:
        - the map takes about 2.4 times as long to build (101168: 21.6 against 9.0 s);
        - local maps with energy above 8 (twice an isometry's) drop by more than half;
        - mean drift is 25% lower on 101168;
        - texture symmetric Dirichlet is 8–22% lower.
      - Do not use `utils::embed_segments` for this. It meshes a background grid and computes
        rational coordinates, and was 100 times slower.
    - Feature curves split a patch into sides, each flattened on its own.
    - Curves that end inside a patch are cut into wedges.
    - Where a vertex is removed on the outline, the after face is split virtually, so both
      sides keep their area.
  - **Vertex moves** (smoothing), by `Smoothing`:
    - `flatten`, the default: the vertex's star before and after, flattened jointly per side,
      so points stay where they were on the surface. A curve vertex slides along the curve by
      arc length.
    - `barycentric`: points keep their face and barycentric coordinates and ride along with
      the vertex. The paper does this.

    Measured 2026-10-22, flatten against barycentric:
    - Barycentric builds the map 1.3–3× faster (101168: 9.0 against 3.4 s; the bunny through
      isotropic remeshing: 5.7 against 2.8 s).
    - Points drift about 3× further: 0.26 against 0.09 edge lengths on the bunny.
    - Transferred textures suffer most. On the remeshed bunny, symmetric Dirichlet 232 against
      10.6 (the input's 4.1) and 266 flipped triangles against 83; through tetwild, 68 against
      5.2.
    - The time saved is at most a few percent of a tetwild run. So flatten stays the default.
    - A tangent-plane projection with a flatten fallback kept flatten's quality at half its
      cost, but saved only 1–8% of a whole tetwild run, and was not worth its complexity.
  - **`relabel`:** a renumbering.
  - **`embed`:** location in space among the sources.
  - **`replace_faces`:** faces with nowhere to go.
- **Warnings.** `build_warnings()`, and the `MapWarnings` a `to_input` / `to_output` call fills
  when given one, say where the map is not a bijection. A call's warnings are its own: mapping
  never writes to the map, so several threads may map through one at once:
  - `not_a_disk`: a patch that cannot be flattened, mapped by relabelling -- or a side a
    collapse removes whole, whose points then have no image;
  - `flat_triangle`;
  - `inverted`: a local map with a non-positive triangle;
  - `merged_faces`: two faces became one;
  - `unlocated`: a point outside every face of its patch;
  - `replaced`: faces of a `replace_faces`.

  `folded_cells()` counts local maps between patches folded over in 3D. They are still
  bijections, but stretched.
- **`Tracking`.** The user-facing wrapper: input points by file triangles or by
  `input_vertex_ids` triplets; output points by triangles or by `output_vertex_ids` triplets.
- **`transfer_fields` / `surface_io`.** Fields of a gmsh input carried onto the output.
  - The files are read and written through `io::read_mesh` / `io::write_mesh`, so they are
    what `wm.read_mesh` reads and `wm.write_mesh` writes, and a plain gmsh `$ElementData`
    field is carried too. A field is a triangle attribute of the `MeshData`: per face, or,
    for `uv` and `normal` (`is_corner_field`, the fields `io::read_mesh` reads per corner),
    three values per corner, corner after corner (`uv`: six columns). A face field keeps its
    type and column labels.
  - Faces take their centroid's value; corners are evaluated just inside their face.
  - Every output face also gets `input_face` (Int64), the input face it maps to, -1 for none.
  - `add_surface` / `write_surface_into_msh` write the result onto a copy of a tet mesh
    (`tet_mesh` / `output_tet_mesh`): its vertices are the tet mesh's by `output_vertex_ids`
    (`output_rows`), a tracked face already in the triangle block (a simwild face tag) takes
    the fields on its row, and any other is a row added after them.
- **The app** (`surface_map.cpp`, spec `surface_map_spec.json`):
  - `info`: what the map holds, its warnings, the output centroids' round trip, and how far
    they lie from their preimages (drift, in output edge lengths);
  - `transfer`: fields onto the output;
  - `map`: user points.

  Without `input`, `transfer` reads the input files from the history's metadata.

  On the memory route (`surface_map(options, meshes)`) `history` is the bytes of a history,
  never a file name; `info` and `transfer` run, `map` is refused. `transfer` needs `input`, a
  key of `meshes`, and returns the output surface as `mesh` (`mesh_from_surface`), and with
  `tet_mesh`, another key, that mesh with the fields as `tet_mesh` (`add_surface`).

## Extending tracking to another application

### On a TriMesh

1. After the mesh is created, `auto rec = mesh.start_history([&](size_t v) { return pos(v); });`
   The position read must be const.
2. Make the application change the surface only through `split_edge`, `split_face`,
   `collapse_edge`, `swap_edge` and `smooth_vertex`. If it has feature curves, enforce the
   feature rules and add them to `rec->history().feature_edges`.
3. Start from the input file. Read with `io::read_triangle_mesh(paths, ..., &provenance)`, then
   save `history::from_input(provenance, welded_positions, rec->history())`
   (`src/wmtk/history/InputHistory.hpp`): the file as read, a `relabel` to the mesh, then the
   recorded edits. `qslim.cpp` is the shortest example.
4. At the end, `consolidate_mesh()`, then `stop_history()`. Write the output by the history's
   last state: consolidated, face ids and corner order as stored.
5. Save the history with `metadata` set to the JSON parameters, through
   `ComponentIO::save_history(h, history_output)`: a file on the file route, the report's
   `history` on the memory route (whose entry point returns `take_result()`).
6. Add `history_output` to the spec with the same name and meaning as in the other
   applications.

### On a TetMesh

1. Derive from `TetOptimizerMesh` if you can: the hooks, `record_surface_history` and
   `finish_surface_history` come with it. Otherwise override `is_tracked_face`,
   `orient_tracked_face` and `tracked_feature_edges` on your `TetMesh`, and call
   `start_surface_history(position)` yourself.
2. Orient tracked faces consistently per sheet. Use the input orientation, the tags (simwild),
   or anything that does not depend on the face's slot.
3. Start recording once the tracked surface exists, after insertion and the first
   consolidation.
4. After anything that changes tracked flags outside the operations -- re-deriving them,
   filters, tag edits -- call `resync_surface_history()`. `remove_tets_by_ids` does it itself.
   Each resync is a `replace_faces` and a warning in the map. Avoid them if you can.
5. Save after the last change to the tet mesh, then write the tet mesh numbered as it is then:
   `output_vertex_ids` are the mesh vertex ids at save time.
6. For an insertion stage: collect per-facet sources keyed by position
   (`m_insertion_sources`), and finish with the input history
   (`finish_surface_history(&input)`).

### What breaks bijectivity

Each of these shows up as warnings, never as a refusal to run:
- topology changes: welds of two surface vertices along a non-surface edge, sheets merging,
  collapses without the link condition;
- surface changes outside the operations;
- patches that are not disks.

For a bijective map, run tetwild with `preserve_topology: true`. It holds both the input
simplification and the tet operations to the link condition. (`simplify_use_link_condition`
is gone; tetwild and triwild warn if a config still sets it.)

What is left under `preserve_topology` (101168: 298 warnings, no welds):
- collapses near a non-manifold vertex the input has and keeps, whose star is not a disk;
- collapses where 4 to 12 feature curves meet. The curves cut the patch into one-face sides
  the joint flattening refuses, and each such face is mapped onto itself by barycentric
  coordinates. That is a bijection on the face, though possibly not consistent with its
  neighbours along the curves; not looked into.

## Extending the mapping component

- **A new edit kind:**
  1. Add it to `EditKind` with its semantics.
  2. Implement it in `Replay::apply`.
  3. Write and read it in `SurfaceHistory.cpp`, bumping the version.
  4. Give `SurfaceMap`'s constructor a case that builds a `Step`, and teach `map()`'s `visit` to
     move points through it, both directions.
  5. Add the warnings it can raise to `MapWarnings`.
- **A new way of naming output points:** if the application's output numbers vertices its own
  way, set `output_vertex_ids`. `Tracking` then speaks in vertex triplets. Anything else --
  tet ids and local faces, say -- belongs in `Tracking`, converting to and from history face
  ids.
- **A new operation:** add it to `surface_map.cpp` and to `operation`'s options in the spec,
  with every new option documented there.
- **Curves (2D, triwild).** Not supported. The history and the map are for triangle complexes;
  tracking curves would need a 1D history and 1D local maps.

## Evaluating correctness

The checks used so far, from cheapest to strongest:
1. **The replay reaches the output.** Replay the history and compare its last state with the
   application's output surface, by sorted corner positions.
   - `test_tetwild_history.cpp`: the exported tracked faces.
   - `test_simwild_history.cpp`: the output's tag interfaces.
   - `test_remeshing_history.cpp`: isotropic.
2. **The recorder recognised everything.** `TrackedSurfaceRecorder::stats()`:
   - `replaced` = 0 and `unknown` = 0;
   - welds = 0 under `preserve_topology`.

   Also: no "feature edges ... are not the mesh's" warning at save.
3. **The map builds without warnings.** `build_warnings().total() == 0`; the `surface_map`
   `info` operation prints them, and the first offending edits.
4. **Round trips.** `to_input(to_output(p)) == p`, and the reverse, below 1e-8 in barycentric
   coordinates. A map that is not a bijection misses by far more. So does one that stretches
   a lot. To tell which, trace one point both ways, step by step, and replay the history
   next to the traces, comparing the two in 3D after each edit. Compare in 3D, not by face
   and barycentrics: a point on an edge has two names.
   - The discrepancy should start at rounding (1e-14).
   - A step that multiplies it by the same factor for every point stretches: that is the map.
   - A step that multiplies it by about the square of a triangle's aspect ratio computes
     badly. 101168 under `preserve_topology` had the embed doing that: barycentric
     coordinates solved through the normal equations, on a sliver of aspect 3.5e4.
   - The scaffolding for this was a temporary `trace()` on `SurfaceMap`. It is not in the
     repository; it records `pts[0]` after each step in `map()`.
5. **A check the map does not see:**
   - simwild's tags: an output interface face maps to an input face between the same two tag
     sets;
   - for surface inputs: onto a file whose tag the interface separates;
   - tetwild: two disjoint spheres from two files, each output face mapping onto the file of
     its sphere.
6. **Geometry.** The image of a point lies near it: within about two input edge lengths in the
   examples, or the envelope for tet applications. A linear attribute remapped through the
   inverse map comes back close to itself.
7. **Quality of the map.** `folded_cells()` counts local maps that must stretch. For UVs,
   compare the distortion of the transferred texture with the input's: flipped texture
   triangles, and the conformal distortion of each triangle's 3D-to-UV map.
8. **Recording changes nothing.** With deterministic (serial) runs, an application with
   recording on, with it off, and current `main` must write byte-identical meshes.
   - Compare instruction counts (`/usr/bin/time -l`) for the cost, not wall time.
   - At 16 threads, runs differ in their iteration counts, which swamps the overhead in wall
     time.

## Folds

Remeshers that do not check normals fold the surface over locally. The map stays bijective
there, but stretches. Two exact checks, both counting folds the same way:
- **`isotropic_remeshing`'s `check_inversions`**, on by default.
- **`TetOptimizerMesh`'s `check_surface_folds`** (tetwild, simwild), off by default.

Both refuse a collapse, swap or smoothing if either of these grows:
- the number of faces not facing the replaced patch's normal, or the new patch's own normal;
- the folds in the stars of the vertices it touches.

Facing is exact: `orient3d` of the triangle and a point below it along the normal
(`utils::predicates::faces_along`).

The tet check (`TetOptimizerMesh::tracked_surface_before/after`, called by `TetMesh`'s
operations through `checks_tracked_surface()`):
- **Before:** the tracked faces around the operation. A collapse takes those at either end;
  a swap, those inside its ring; a smoothing, those at the vertex. It records the patch's
  summed area vector, how many faces do not face along it, and the star folds over the
  patch's vertices. It skips operations with no surface vertex.
- **After**, once the after-hook and invariants have passed: the same counts on the faces
  there. More of either refuses the operation, like a failed after-hook, with a rollback.
- **Splits** are not checked: they halve faces and move nothing.
- **Colored smoothing is off** while the check is on. Colored smoothing moves vertices of a
  class concurrently without locks, and the check reads the stars of a vertex's neighbours,
  which two vertices of one class can share.
- **Stars come from the vertex's own tets;** see *Locking*.

What it does, from `test_surface_folds.cpp` (8 threads, 10 iterations, three runs each).
These are folded star faces after insertion and at the end; insertion already leaves many,
creases sharper than their star's average included:

| model | check off, at the end | check on, at the end | after insertion |
|---|---|---|---|
| bunny | 9–12 | 0–1 | 122–143 |
| crown | 35–77 | 13–31 | 1556–1801 |
| thingi_100036 | 87–117 | 15–25 | 195 |
| 37989_sf | 1851–1902 | 1681–1697 | ~3500 |
| Octocat | 108–117 | 65–73 | 158–199 |

On the octocat without simplification, folded local maps went from 3,456 to 112, for +5%
final max energy. Keep it off by default (the user's decision, 2026-10-21).

Tests:
- `tetwild-surface-folds-refused`: a vertex moved past its star's outline is refused.
- `tetwild-surface-folds-none-made`: a parallel bunny run ends with no more folds than the
  insertion left.
- `[.folds-compare]` (hidden): the table above.

## Costs

Measured on the 15 challenging tetwild models (default parameters, so `preserve_topology`
off), serially, comparing recording on, recording off, and `main` (43d74ecfe0). Scripts:
`exp_challenging/run_eval.py` and `reinfo.py` in the session scratchpad, not in the repository.
- **Recording off costs nothing.** Output meshes are byte-identical to `main`'s on all 15
  models, with the same iteration counts. Instructions retired: -0.07% to +0.06% per model,
  -0.04% in total. The hooks are one pointer test per operation.
- **Recording on:** +0.7% to +8.2% instructions per model, +4.4% in total. Wall time
  +6% in total. Peak memory +1% to +39%; the most is on 46024, whose 320k-face input the
  history keeps as read.
- **History size:** 0.5 to 38 MB; 35 to 80 bytes per edit, plus the input as read.
  - 25.6 MB (750k edits) for the longest run, 1017020;
  - 38 MB for 46024, mostly its input (190 bytes per edit).
- **Building the map:** 0.4 to 31 s, about a second per 25,000 edits.
- **Map quality with `preserve_topology` off:**
  - there are always some not-a-disk and merged-face warnings (welds, sheets merging);
  - 13 of the 15 models round-trip below 1e-3;
  - 1017012 and 1017020 (3,000 and 4,450 welds) lose 5 and 9 output centroids and round-trip
    up to the model's scale.

Check memory by peak footprint (`/usr/bin/time -l`). At 16 threads, wall times vary with the
iteration counts, and other load moves them too. Compare instruction counts.

## Where things are

| what | where |
|---|---|
| history, edit kinds, file format, composition | `src/wmtk/history/SurfaceHistory.{hpp,cpp}` |
| replay | `src/wmtk/history/Replay.{hpp,cpp}` |
| tracked-surface recorder | `src/wmtk/history/TrackedSurface.{hpp,cpp}` |
| TriMesh hooks | `src/wmtk/TriMesh.{h,cpp}` (`start_history`, `commit_history`) |
| TetMesh hooks | `src/wmtk/TetMesh.{h,cpp}`, `TetMeshEdgeSplittingConn.cpp`, `TetMeshEdgeCollapsingConn.cpp`, `TetMeshSwapMeshConnectivity.cpp` |
| TetOptimizerMesh side, fold check | `src/wmtk/TetOptimizerMesh.{h,cpp}`, `OptimizerParameters.h` |
| read provenance | `src/wmtk/io/read_triangle_mesh.{hpp,cpp}` |
| insertion sources | `src/wmtk/utils/EmbedTriangles.{hpp,cpp}` (`facet_triangles`) |
| map, transfer, tracking, app | `components/surface_map/wmtk/components/surface_map/` |
| isotropic recording | `components/isotropic_remeshing/.../isotropic_remeshing.cpp` |
| tetwild recording | `components/tetwild/.../tetwild.cpp`, `VolumemesherInsertion.cpp` |
| simwild recording | `components/simwild/.../simwild.cpp`, `EmbedSurface.cpp`, `read_image_msh.cpp`, `SimWildMesh.cpp` |
| tests | `tests/test_history.cpp`; `test_surface_map.cpp`, `test_tracking.cpp`; `test_remeshing_history.cpp`; `test_tetwild_history.cpp`, `test_tetwild_tracking.cpp`; `test_simwild_history.cpp`, `test_simwild_tracking.cpp` |
