# Histories

A remeshed surface has new vertices and new triangles, and data defined on the input (labels,
texture coordinates, points a user cares about) has no obvious place on it. A *history* records
every change the component made to its tracked surface, from the input files as read to the
output. From it the `tracking` component rebuilds a map between the input surface and the output
surface, in both directions, and carries points and fields across. The same holds one dimension
down for triwild's curves, and, around the surface, for the volume of a tet mesh.

This page is the overview. The design notes, [Tracking: recording, mapping, extending](../tracking.md),
have the full rules; the [tracking tutorial](../tutorials/tracking.md) shows a run.

## Recording

The option is [`history_output`](../reference/tetwild.md#history_output), in every component
that has it:

| Component | Records | From | To |
|---|---|---|---|
| `isotropic_remeshing`, `qslim`, `shortest_edge_collapse` | the surface | the input file as read | the output file |
| `tetwild` | the tracked surface of the tet mesh | the input files as read | the tracked faces of the output, after any filter |
| `simwild` (3D) | the interfaces between regions | the input files, or the tagged mesh it starts from | the interfaces of the output |
| `triwild` | the tracked curves | the input curve files as read | the tracked edges of the output, after any filter |

[`volume_history_output`](../reference/tetwild.md#volume_history_output) (tetwild and simwild in
3D, with `history_output`) also records the tetrahedra around every operation that changes the
tracked surface, so that the volume on each side of the surface can be mapped as well.

Recording happens inside the operations. Each operation commits an *edit* at its commit point,
after its hooks and invariants have accepted it and while it still holds its locks, so the edits
of a parallel run are appended in an order that replays correctly. This is the commit of a 2D
edge split
([`TriMesh.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriMesh.cpp)):

```cpp
    if (m_history) {
        history::Edit e{history::EditKind::split_edge, {vid1, vid2, new_vid}, {}, {}};
        for (size_t i = 0; i < fids.size(); ++i) {
            e.faces.push_back(fids[i]);
            e.faces.push_back(new_fids[i]);
        }
        commit_history(std::move(e));
    }
```

A refused operation records nothing, since it is rolled back before its commit point. Recording
changes nothing the operations do.

The tracked surface of a tet mesh has no stable face ids, so the recorder gives it ids of its own
and records each operation as the difference between the tracked faces around it before and
after. The stages before the optimization are recorded into the same history: the weld of the
input files, the [simplification](embedding.md#2-simplify-the-input), and the
[insertion](embedding.md#3-the-background-mesh-and-the-arrangement), which records, for every
face of the arrangement that lies on the input, the input triangles it lies on. A filter's
removal of tracked faces is recorded too, as faces no map can follow.

## What a history holds

- **A snapshot** of the complex it starts from: vertices, faces (triangles, or segments for
  curves) and their positions.
- **Features**: feature edges of a surface, feature points of curves, which the edits keep by
  fixed rules and the map takes onto themselves.
- **The edits**, in commit order. Each names vertex and face ids and carries the new position of
  every vertex it moved:

| Edit | Records |
|---|---|
| `split_edge`, `split_face` | a new vertex on an edge or in a face, and the faces it split |
| `collapse_edge` | the removed vertex and the one it was merged into |
| `swap_edge` | the edge and the two faces it flipped |
| `move_vertices` | new positions only (smoothing) |
| `consolidate` | a renumbering of the live vertices and faces |
| `relabel` | a renaming of vertices and faces: the weld, a reordering |
| `embed` | the complex replaced by one lying on it, with the sources of each new face: the insertion |
| `replace_faces` | faces removed or added that no other edit describes; the map cannot follow them |

- **Metadata**: the component's options, as JSON, which is how `tracking` finds the input files
  when it is not given them. Components that number their output vertices their own way also
  store the output's vertex ids.

The file is binary: the magic `WMTKHIST`, a format version (7), the number of corners of a face
(3 for a surface, 2 for curves), then varint-encoded records
([`History.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/history/History.cpp)).
On the memory route no file is written: the report holds the bytes of the file as `history`, and
`wm.write_history` saves them.

## What the tracking component rebuilds

The [`tracking`](../reference/tracking.md) component replays the edits and builds one local map
per operation, between the patch the operation replaced and the patch it left:

- a split maps each new face exactly, by an affine map onto the face it came from;
- a collapse or a swap flattens the patch before and after jointly, on a shared boundary, and
  maps between the two flattenings;
- smoothing flattens the star of the moved vertex before and after;
- on curves, each chain of segments before is mapped onto the chain after by arc length.

The composition of the local maps is the map between input and output. Where every local map is a
bijection, so is the whole map; where one is not (a patch that cannot be flattened, a face a
filter removed), the component reports it. The [`operation`](../reference/tracking.md#operation)
selects what to do with the map: `info` reports on it, `map` maps points (`face b0 b1 b2` on a
surface input, `v0 v1 v2 b0 b1 b2` on a tet mesh output), `transfer` carries the fields of the
input's triangles onto the output, `features` follows triwild's feature points, and `volume` and
`map_volume` do the same through the volume history.

## See also

- [Tracking: recording, mapping, extending](../tracking.md): the complete notes, including how to
  record a new application and how the map's correctness is measured.
- The [tracking tutorial](../tutorials/tracking.md).
- [In-memory I/O](../in-memory-io.md) for histories on the memory route.
