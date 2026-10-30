# Embedding

tetwild, triwild and simwild do not require a clean input. A tetwild input is a triangle soup that
may intersect itself, have holes, duplicates and non-manifold edges; a triwild input is a set of
polylines that may cross and overlap. Before the [optimization](quality.md), the input is inserted
exactly into a background mesh that covers its bounding box: every input triangle (or segment)
ends up as a union of faces (or edges) of the mesh, which are tracked from then on. This page
describes the steps from the input files to that mesh, and the filter that removes the outside at
the end.

## 1. Read and weld

The input files are read and concatenated. Vertices closer than
[`remove_duplicate_eps`](../reference/tetwild.md#remove_duplicate_eps) (a fraction of the bounding
box diagonal) are merged; 0 merges only vertices that coincide exactly. The bounding box of the
input sets the scale: \(\varepsilon\) and the target edge length are relative to its diagonal (see
[Robust geometry](robust-geometry.md#envelopes)).

## 2. Simplify the input

An input often has far more detail than the target edge length: thousands of tiny triangles on a
flat region, tessellated curves. Inserting all of it would give the optimizer a mesh much finer
than asked for, which it would first have to coarsen. So the input is simplified first, inside an
envelope of its own:

- In 3D, tetwild simplifies the input surface, a `TriMesh`, by shortest edge collapse, with the
  envelope as invariant, at `simplify_envelope_ratio` \(\cdot\,\varepsilon\) (half of
  \(\varepsilon\) by default). The link condition applies only with
  [`preserve_topology`](../reference/tetwild.md#preserve_topology), so by default the
  simplification may change the topology of the surface. The open boundary may move within a tube
  around the input's boundary ([`simplify_boundary_envelope`](../reference/tetwild.md#simplify_boundary_envelope)).
  simwild does the same, except under `preserve_topology`, where it simplifies after the
  insertion instead.
- In 2D, triwild collapses the shortest segments of the curves while they stay inside the
  simplification envelope.

The tighter envelope leaves the optimizer room: the simplified surface is within half of
\(\varepsilon\) of the input (by default), and the optimizer, which is held to the full
\(\varepsilon\) around the original input, starts with room to move.
[`skip_simplify`](../reference/tetwild.md#skip_simplify) inserts the input as it is.

## 3. The background mesh and the arrangement

**In 3D** (tetwild, and simwild on surfaces), the background is a Delaunay tetrahedralization of
the simplified input's vertices, the eight corners of the input's bounding box grown by a
fifteenth of its diagonal, and a regular grid of points spaced at most a twentieth of the diagonal
apart, without the grid points closer to the input than half of that
([`DelaunayBoxMesh.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/DelaunayBoxMesh.hpp)).

The input triangles are then inserted with VolumeRemesher's `embed_tri_in_poly_mesh`, which
computes the exact arrangement of the triangles and the background tetrahedra, and returns
tetrahedra that conform to every input triangle
([`EmbedTriangles.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/EmbedTriangles.hpp)).
Intersections between input triangles are resolved there, exactly. The remesher's tetrahedra are
used directly, with no extra vertices at cell centres. The vertices have rational coordinates;
how they become doubles is described in
[Robust geometry](robust-geometry.md#exact-and-rounded-positions). tetwild's feature curves and
points ([`input_edges`](../reference/tetwild.md#input_edges),
[`input_points`](../reference/tetwild.md#input_points)) are inserted in the same arrangement, so
each feature edge is tiled by tetrahedron edges and each feature point is a vertex.

**In 2D** (triwild, and simwild on curves), VolumeRemesher's `embed_seg_in_tri_mesh` computes the
exact arrangement of the segments: a triangulation of the input's bounding box, seeded with a grid
of background points, in which every input segment is a chain of triangle edges and every crossing
of two segments is a vertex
([`EmbedSegments.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/EmbedSegments.hpp)).
The coordinates of a crossing are rational and are kept as such: two distinct crossings can round
to the same doubles.

## 4. The tracked surface

The faces of the arrangement that lie on an input triangle form the *tracked surface* (in 2D, the
edges on an input segment form the tracked curves). Which faces they are comes from the
arrangement's provenance, the record of which output faces tile each input triangle, not from a
geometric test after the fact. Each face (or edge) carries a few tags
([`SurfaceTagAttributes.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/SurfaceTagAttributes.h)):

```cpp
    /// Is this simplex part of the tracked surface.
    bool m_is_surface_fs = false;
    /// Which bbox side this simplex is on; -1 for none.
    int8_t m_is_bbox_fs = -1;
```

and an orientation (the signed number of input sheets lying on it) and an input set (which inputs
it lies on). The operations carry these through every split, collapse and swap. The optimization
keeps the tracked surface inside the [envelope](robust-geometry.md#envelopes) and the faces on the
bounding box on it, so the box does not collapse.

## 5. The outside: winding numbers and `filter`

The mesh covers the whole bounding box. After the optimization, the
[`filter`](../reference/tetwild.md#filter) option of tetwild and triwild decides what to keep:

| `filter` | Keeps |
|---|---|
| `none` (default) | Everything. |
| `input` | The cells inside the input: the winding number of the input, evaluated at each cell's centroid, is above 1/2. |
| `tracked` | The same with the winding number of the tracked surface, oriented as the input was. Coincident surfaces add up and opposite ones cancel. Each component must be closed. |
| `flood` | The tracked surface divides the mesh into parts, and the part that touches the bounding box most is removed. The input must be closed; otherwise the whole mesh is one part and nothing is left. |

The winding number is the generalized winding number, so it is defined for open and
self-intersecting inputs too: 1 inside a closed, consistently oriented input, 0 outside, and
fractional near holes
([`WindingNumber.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/WindingNumber.hpp)).
`input` judges each cell by the input rather than by the surface the mesh conforms to, so it can
misclassify cells along the surface and leave it wrinkled; `tracked` judges by the tracked surface
itself. The winding numbers are also written to the output as fields (tetwild writes one per
input, `wn_<name>`); [`skip_winding_number`](../reference/tetwild.md#skip_winding_number) leaves
them out when no filter needs them.

triwild uses the per-input winding numbers to give each triangle the [regions](regions.md) of the
inputs it lies inside. simwild has no filter: it keeps the whole box and tags every cell with the
inputs that contain it (see [Regions](regions.md)).

With [`history_output`](../reference/tetwild.md#history_output), the whole sequence above is
recorded, from the files as read to the output: the weld, the simplification, the insertion, the
optimization and the filter (see [Histories](histories.md)).

## See also

- The [tetwild](../tutorials/tetwild.md) and [triwild](../tutorials/triwild.md) tutorials.
- [Robust geometry](robust-geometry.md) for rational coordinates and the envelope.
- [VolumeRemesher](https://github.com/wildmeshing/VolumeRemesher), the exact arrangement both
  dimensions use.
- Hu et al. 2018, [Tetrahedral Meshing in the Wild](https://cims.nyu.edu/gcl/papers/2018-TetWild.pdf),
  and Hu et al. 2019, [TriWild](https://cims.nyu.edu/gcl/papers/2019-TriWild.pdf).
