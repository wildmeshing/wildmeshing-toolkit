# TetWild with features

A tet mesh for a simulation often has to contain more than the surface: a curve along which a
load is applied or a crack runs, a point where a sensor sits or a boundary condition holds. With
[`input_edges`](../../reference/tetwild.md#input_edges) and
[`input_points`](../../reference/tetwild.md#input_points), [TetWild](../tetwild.md) inserts such
curves and points into the tet mesh exactly, in the same arrangement as the surface: each feature
edge becomes a chain of tet edges, each feature point a tet vertex. The optimization then keeps
them: the curves stay in a thin tube around the input, and the points and the ends of the curves
stay within the tube's radius of where they were.

This page runs it on a small test case and checks what was kept.

## The data

Three files from the example data:

- `tetwild_features_cube.obj`: the surface, a cube from (0, 0, 0) to (2, 2, 2), 12 triangles.
- `tetwild_features_edges.obj`: a feature curve of two edges, from (0.3, 0.3, 0.3) to the cube's
  centre (1, 1, 1) and on to (1.7, 1, 1.7), and one vertex, (0.5, 1.5, 0.5), that no edge uses.
  A vertex of an edge file without an edge is a feature point.
- `tetwild_features_points.obj`: two feature points, (1, 0.5, 1.5) inside the cube and (2, 1, 1) on
  its face \(x = 2\). Only the vertices of a point file are used.

The curve and the points are inside the volume, not on the surface: nothing in the surface would
make the mesh conform to them.

## The run

The inside of the cube (`filter` `tracked`), with every tet's energy below 10, and sequentially
(`num_threads` 0) so that the numbers below repeat exactly:

=== "Command line"

    ```json
    --8<-- "docs/examples/json/workflow_tetwild_features.json"
    ```

    ```bash
    wmtk_app -j workflow_tetwild_features.json
    ```

    The input paths are relative to the JSON file; the outputs, `cube_final.msh` and
    `cube_features.obj` among them, are written to the working directory.

=== "Python"

    ```python
    --8<-- "docs/examples/python/workflow_tetwild_features.py:run"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/workflow_tetwild_features.cpp:run"
    ```

    Link `wmtk::tetwild`.

The result has 4,857 vertices and 24,335 tets, with the largest energy 5.50, after one iteration.
It takes a few seconds.

## What is kept

**The curve.** The insertion tiles the two input edges with 14 tet edges, and tags them. The
optimization may split, collapse and move tagged edges like any other, as long as they stay in a
tube around the input curve, of radius
[`feature_envelope_ratio`](../../reference/tetwild.md#feature_envelope_ratio) (0.5) times
\(\varepsilon\): 0.00173 here, with \(\varepsilon\) = 0.00346. In this run the 14 edges stay on the
input curve, up to rounding. The bend at the cube's centre is a vertex of the curve like any other:
it is not anchored, and the tube is what keeps the curve's shape there.

**The points.** Five points are anchored: the curve's two ends, the free vertex of the edge file,
and the two points of the point file. Each is a vertex of the output at exactly its input position.
The optimization may move an anchored vertex within the tube's radius of its point, and refuses a
collapse that would leave a point without a vertex within it
([`preserve_feature_points`](../../reference/tetwild.md#preserve_feature_points)).

**The curve network's topology.** The run compares the Euler characteristic of each connected
curve of the input with that of the tagged edges, and warns when they differ. Here both are one
open curve.

## Checking it

**In the log.** The run logs what it inserted and anchored, and the Euler characteristics of the
curves; [`DEBUG_hausdorff`](../../reference/tetwild.md#DEBUG_hausdorff), set in the JSON file, adds
the curves' deviation, and
[`DEBUG_feature_retention`](../../reference/tetwild.md#DEBUG_feature_retention) the number of
anchors kept:

```text
feature anchors: 2 curve endpoints, 0 junctions; anchoring 5 points in total (junction cleanup on)
features after insertion: 2 edges tiled by 14 tet edges, 3/3 points are output vertices
feature deviation: 14 tagged edges | containment d(tagged->input) = 3.846e-16 | tube = 0.001732
feature deviation: coverage d(input->tagged) = 0 (diagnostic; large means part of an input curve is no longer represented)
Euler characteristic, feature curves: input [1] | tagged output [1]
feature points retained: 5/5
```

Containment is the largest distance from the tagged edges to the input curve, which the tube
bounds. Coverage is the other direction, the largest distance from the input curve to the tagged
edges, which nothing bounds: it grows when a curve is shortened. Both are measured before the
filter. The "3/3 points" are the points inserted on their own, the free vertex and the two points
of the point file; the curve's ends are vertices of the curve.

**In the output.** `cube_features.obj` holds the tagged edges (`l` lines) and the anchored vertices
(`p` lines), at their output positions. In memory, the same is `outputs["features"]`, with an
`edge` and a `point` block. Compare them with the input:

```python
--8<-- "docs/examples/python/workflow_tetwild_features.py:check"
```

```text
anchors: the worst is 0 from a vertex (tube radius 0.0017)
curve: tagged edges within 3.8e-16 of it, covering it to 0
```

With threads the run changes slightly: two runs with 4 threads gave 25,571 and 25,183 tets, with
14 and 17 tagged edges, and in one of them the tagged edges moved up to 0.00069 off the curve,
inside the tube. The anchors were at distance 0 in both.

## Without the anchors

[`preserve_feature_points`](../../reference/tetwild.md#preserve_feature_points) false keeps the
curve in its tube but anchors nothing:

```python
--8<-- "docs/examples/python/workflow_tetwild_features.py:erosion"
```

```text
preserve_feature_points false: the worst anchor is 0.11 from a vertex, the curve is covered to 0.18
```

The points' vertices moved or were collapsed away: the farthest point is now 0.11 from the nearest
vertex, 64 times the tube's radius. And the curve lost 0.18 of its length at one end: its last
tagged vertex is at (1.574, 1, 1.574) instead of (1.7, 1, 1.7). A collapse along the curve that
shortens it keeps what is left inside the tube, so the tube alone cannot see it; that is what the
anchors at the curve's ends are for.

## Options

- [`feature_envelope_ratio`](../../reference/tetwild.md#feature_envelope_ratio) (0.5): the tube's
  radius, as a fraction of \(\varepsilon\). Here 1.0 gives the same mesh: the curve did not use the
  room.
- [`allow_junction_cleanup`](../../reference/tetwild.md#allow_junction_cleanup) (true): where three
  or more feature edges meet, the junction is not anchored, and may merge and move within the tube;
  every curve through it stays tagged. False anchors junctions too.
- [`filter`](../../reference/tetwild.md#filter): a feature in a region the filter removes leaves
  the tet mesh with it, but stays in the feature output. With an extra point at (2.5, 1, 1),
  outside the cube, the nearest vertex of the filtered mesh is 0.5 away, while the feature output
  holds the point.
- The feature curves and points are part of the input's bounding box. The extra point above grows
  the box, and with it \(\varepsilon\) (0.00377 instead of 0.00346) and the edge length: the same
  run gives 16,867 tets instead of 24,335.

## See also

- The [TetWild tutorial](../tetwild.md), and the reference of
  [`input_edges`](../../reference/tetwild.md#input_edges),
  [`input_points`](../../reference/tetwild.md#input_points) and
  [`preserve_feature_points`](../../reference/tetwild.md#preserve_feature_points).
- [Embedding](../../concepts/embedding.md): how the curves and points are inserted with the
  surface.
- [TriWild](../triwild.md#feature-points-input_points-preserve_feature_points): the same anchors in
  2D.
