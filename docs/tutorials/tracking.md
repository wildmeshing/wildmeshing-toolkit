# Tracking: correspondences between input and output

Remeshing and meshing replace a surface with another one. `tracking` keeps the two connected: an
application records every operation it commits (`history_output`), and `tracking` rebuilds from
that record a bijective map between the input surface and the output, one local map per
operation, composed in order. Through the map, any point of the input has a point on the output,
and the other way round.

This tutorial takes the dense Stanford bunny (69,451 faces) through two applications,
[isotropic remeshing](isotropic_remeshing.md) down to about a thousand faces and then
[TetWild](tetwild.md), and maps every vertex from the dense surface onto the tet mesh's surface
and back. That is what it takes to carry data — a texture, a field, boundary conditions — between
a scanned surface and the mesh a simulation runs on.

The [design notes](../tracking.md) explain how the map is built, how to record a new application,
and how to check a map; [Concepts: histories](../concepts/histories.md) is the short version.

## The script

The whole pipeline is one script, `docs/examples/python/tracking_correspondences.py`:

```bash
python docs/examples/python/tracking_correspondences.py           # opens a viewer (polyscope)
python docs/examples/python/tracking_correspondences.py --no-gui  # statistics only
```

It needs the `wildmeshing` package and NumPy; the viewer also needs `polyscope`. The sections
below go through it. It is also an integration test of the toolkit
(`tests/python/test_tracking_correspondences.py`): the numbers on this page are what it checks.

## Record the history

On the memory route, any non-empty `history_output` records the history, and the report returns
it as bytes. `wm.Tracking` builds the map from them:

=== "Python"

    ```python
    --8<-- "docs/examples/python/tracking_correspondences.py:coarsen"
    ```

=== "C++"

    ```cpp
    --8<-- "docs/examples/cpp/tracking.cpp:record"
    ```

    ```cpp
    --8<-- "docs/examples/cpp/tracking.cpp:map"
    ```

    Link `wmtk::isotropic_remeshing` and `wmtk::tracking`.

=== "Command line"

    On files, `history_output` names the file the history goes to, and the `tracking` component
    reads it back. These three files run in order in one directory, with `bunny.obj` and
    `points.txt`:

    ```json
    --8<-- "docs/examples/json/tracking/1_remesh.json"
    ```

    ```json
    --8<-- "docs/examples/json/tracking/2_info.json"
    ```

    ```json
    --8<-- "docs/examples/json/tracking/3_map.json"
    ```

    `points.txt` lists points on the input as a face and barycentric coordinates; the map writes
    each one's image on the output, in the same form, to `points_on_coarse.txt`:

    ```text
    --8<-- "docs/examples/json/tracking/points.txt"
    ```

With `eps_rel` 0.02 and `length_rel` 0.045, the bunny comes out with 1,041 faces. The map's faces
are exactly the input's and the output's, as given:

```python
--8<-- "docs/examples/python/tracking_correspondences.py:check_map"
```

`build_warnings` counts the places where a local map is not a bijection. For this remeshing it
is zero everywhere. `operation: info` on the command line reports the same and checks the map:
here it is built from 38,190 edits (572 splits, 34,779 collapses, 254 swaps, and the vertex
moves), and every output face centroid maps to the input and back to within 6e-17.

## Chain a second application

TetWild's history starts at the surface it is given, exactly as given, so it continues where the
remeshing's ends: mapping through one map and then the other is the map of the two histories
concatenated.

```python
--8<-- "docs/examples/python/tracking_correspondences.py:tetwild"
```

The options that keep this map a bijection:

- [`skip_simplify`](../reference/tetwild.md#skip_simplify): the surface is already as coarse as
  wanted, so TetWild does not simplify it again.
- [`preserve_topology`](../reference/tetwild.md#preserve_topology) and
  [`check_surface_folds`](../reference/tetwild.md#check_surface_folds): the tracked surface keeps
  its topology and does not fold over. Without them the map has tens of warnings.
- [`filter`](../reference/tetwild.md#filter): `input` keeps the tets inside the surface by its
  winding number. The bunny is open (its bottom has holes), so neither flood fill nor the tracked
  surface's winding number applies. The filter can remove a tracked triangle or two, when both of
  its tets are outside; the map counts each as `replaced`, and the few points on them have no
  image. `filter: none` keeps every tet, and every point maps.

On this run TetWild gives 5,897 tets and 1,070 tracked triangles, with one map warning, a
replaced triangle.

TetWild logs `Output topology is not the same as the input topology!` here, comparing Euler
characteristics: the input surface is open (−3), and the boundary of the tets kept is closed (2).
That is expected for an open input.

## Map points

A point is a face and barycentric coordinates in it. To map a vertex, the script names it as a
corner of one of its faces. The tet mesh's tracked triangles are named by their three vertices;
`triangle_ids` finds them among the tet mesh's surface triangles.

```python
--8<-- "docs/examples/python/tracking_correspondences.py:map_forward"
```

And back, from the coarse surface and from the tet mesh's surface onto the dense input:

```python
--8<-- "docs/examples/python/tracking_correspondences.py:map_back"
```

What comes out, as distances from each point to its image, relative to the bounding-box
diagonal:

| Direction | Unmapped | Mean | Max |
|---|---|---|---|
| dense → coarse | 0 | 6.1e-3 | 5.4e-2 |
| coarse → dense | 0 | 3.0e-3 | 2.3e-2 |
| dense → tet surface | 2 | 6.2e-3 | 5.4e-2 |
| tet surface → dense | 0 | 3.3e-3 | 2.3e-2 |

1,113 of the bunny's 35,947 vertices are in no face; they are not on the surface and
are skipped. The two dense vertices without an image lie on the triangle the filter removed.

Most distances are below the envelope (2e-2 of the diagonal), but the map is not a closest-point
projection: it follows each operation, so a point can also slide along the surface, and some
images are farther than the envelope. Mapping a point there and back returns it, to within rounding.

<div>
<wmtk-viewer example="tracking"></wmtk-viewer>
</div>

The dense input painted with a checkerboard on the left; on the right the tet mesh's surface, each triangle painted with the color found at its centroid's preimage through both maps: drag to rotate, scroll to zoom. Every example is in the [gallery](../gallery/index.md).

## How coarse

Face count goes roughly with 1 / `length_rel`², until the envelope stops the collapses. On the
bunny:

| `eps_rel` | `length_rel` | faces |
|---|---|---|
| 1e-2 | 0.02 | about 4,150 |
| 1e-2 | 0.04 | about 1,620 |
| 1e-2 | 0.05 | about 1,425 |
| 2e-2 | 0.04 | about 1,200 |
| 2e-2 | 0.045 | about 1,080 |

TetWild then splits and collapses the surface a little within its own envelope.

## Threads

The remeshing and TetWild run with 8 threads. With more than one thread, the order of the
operations depends on the schedule, and the result changes slightly from run to run: the counts
above move by a few faces. `--threads 0` gives identical results every time.

## The `tracking` component

On files, the `tracking` component does with a history file what `wm.Tracking` does in memory,
by [`operation`](../reference/tracking.md#operation):

- `info`: build the map, report its warnings, and check it on the output's face centroids.
- `map`: map the points of a file (`points`) to the output or back (`direction`).
- `transfer`: carry every field of the input's triangles onto the output surface; with
  `output_obj`, write a textured OBJ with `uv_field` as its texture coordinates.
- `features`: for curves (TriWild), where each feature point of the input ended up.
- `volume`, `map_volume`: the same through a tet mesh's volume, from the volume history that
  TetWild and SimWild record with `volume_history_output`.

## See also

- [Reference: every option of `tracking`](../reference/tracking.md)
- [Tracking design notes](../tracking.md): the history format, recording an application,
  checking a map
- [Component I/O](../getting-started/calling-components.md): histories on the memory route
