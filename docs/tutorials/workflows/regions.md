# Multi-region pipeline

A mesh for a simulation with several parts is built in steps: mesh the parts together, decide what
the overlaps are, add the layers the simulation needs, and restore the quality the edits cost. Each
step is a component call whose input is the previous step's output; the regions travel with the
mesh, in the `tags` attribute of its cells (see [Regions](../../concepts/regions.md)).

This page runs such a pipeline in memory, from Python and from C++, on two overlapping spheres:

1. [SimWild](../simwild.md) meshes the two surfaces into one tet mesh, regions `a`, `b` and their
   overlap.
2. SimWild's `replace_tags` gives the overlap to `b`.
3. [Topological offset](../topological_offset.md) carves a gap of thickness 0.01 around `a`, a
   region of its own named `gap`.
4. SimWild's `remeshing` brings every tet back below the target energy.

Each step saves a checkpoint with `wm.save`, so a later step can be rerun from it.

## The pipeline

The whole script is `docs/examples/python/workflow_regions.py`; it runs in well under a minute,
most of it in step 4. The numbers below are from one run with 4 threads; another run gives counts
a few percent different.

### 1. Mesh the surfaces

```python
--8<-- "docs/examples/python/workflow_regions.py:mesh"
```

This is the first run of the [SimWild tutorial](../simwild.md#a-first-run): 20,805 tets, all below
energy 10.

```text
1. simwild: 20805 tets {'a': 1575, 'a & b': 2985, 'ambient': 14629, 'b': 1616}
```

`wm.save` writes `regions/m1.msh` and, since the result has an
[envelope](../simwild.md#the-envelope-a-run-returns), `regions/m1_envelope.msh`.

### 2. Settle the overlap

A cell in both `a` and `b` would be in both materials. `replace_tags` takes `a` away from those
cells and leaves `b`, so `b` is the whole second sphere and `a` the first one minus the lens:

```python
--8<-- "docs/examples/python/workflow_regions.py:edit"
```

```text
2. replace_tags: {'a': 1575, 'ambient': 14629, 'b': 4601}
```

No cell changes, only its regions: the mesh is the one of step 1, with the same quality. A region
edit returns no envelope, so `regions/m2_envelope.msh` is not written. To split the overlap between
the two instead, see [below](#splitting-the-overlap-instead).

### 3. Carve the gap

`topological_offset` with a single region name, `a`, grows a band of cells outward from `a`'s
boundary and gives them the tags in `offset_output_tags`, in place of the ones they had: the band
is cut out of `b` and of the background.

```python
--8<-- "docs/examples/python/workflow_regions.py:offset"
```

```text
3. topological_offset: 30949 tets {'a': 1575, 'ambient': 17499, 'b': 6015, 'gap': 5860}
```

The tet count grows because the band is built by splitting cells: first around `a`, so that no
edge, face or tet outside it has all its vertices on it, then every edge that leaves `a`, at the
target distance; the band is the cells between `a` and these new vertices. The log reports the
largest distance this can reach on this mesh, `maximum marchable distance 0.0374186`, and that the
target, 0.01, is below it: every vertex of the gap's outer boundary is at 0.0099 to 0.0100 from
`a`. A target above it would be cut short at half that distance.

`optimize_offset` false stops there, in a fraction of a second. The default runs the offset's own
optimization, which moves the whole outer boundary, triangles included, onto the target distance;
on meshes from step 1 or 2, with several selections and targets, it had not converged after two to
four minutes, when the runs were stopped (see the
[topological offset tutorial](../topological_offset.md)). The construction leaves thin cells: the
largest energy (`amips` on the output's tets) is 160 to 172 in two runs.

### 4. Remesh

The offset returns no envelope either. Without one, simwild builds the envelope from the
interfaces of the mesh it is given, which is what is wanted here: the gap's walls are new, and no
earlier envelope contains them. `eps_rel` 0.001 (an envelope of 0.0032, a third of the gap) keeps
them close to where the offset put them.

```python
--8<-- "docs/examples/python/workflow_regions.py:remesh"
```

```text
4. simwild remeshing: 105244 tets, max energy 9.88 {'a': 23277, 'ambient': 41556, 'b': 20200,
'gap': 20211}
```

Every tet is below 10 again. The gap is 0.0081 to 0.0149 thick (median 0.0100), measured from the
vertices of its outer boundary to `a`'s boundary: each wall may move by up to the envelope. The
mesh ends five times larger than step 1's, as a well-shaped tet inside the gap is no larger than
the gap is thick. The envelope trades size for fidelity: in a run of the same pipeline, `eps_rel`
0.002 gave 66,351 tets and a gap 0.0068 to 0.0207 thick, against 111,580 tets and 0.0081 to 0.0152
at 0.001.

The first line of the section reads step 3's checkpoint back: `wm.load` returns what `wm.save`
wrote, so commenting out steps 1 to 3 reruns step 4 alone on the same mesh.

### The same in C++

```cpp
--8<-- "docs/examples/cpp/workflow_regions.cpp:run"
```

Link `wmtk::simwild` and `wmtk::topological_offset`. In one run, step 1 gave 20,537 tets, step 3 a
gap of 5,911 tets, and step 4 96,281 tets with the largest energy 9.94. In C++ there are no
checkpoint helpers: `wmtk::io::write_mesh` writes a mesh, and an envelope, if any, is the
mesh `outputs.at("envelope")`.

## Splitting the overlap instead

`resolve_overlaps` gives each cell of the overlap to the nearer of the two spheres, so the
interface between `a` and `b` runs through the middle of the lens. It splits the cells it cuts and
leaves slivers, which a remeshing removes before anything else runs on the mesh:

```python
--8<-- "docs/examples/python/workflow_regions.py:resolve"
```

```text
2'. resolve_overlaps: {'a': 3998, 'ambient': 14629, 'b': 4036} max energy 76920.05
    remeshed: max energy 5.38 {'a': 3187, 'ambient': 14728, 'b': 3175}
```

The rest of the pipeline is unchanged. The largest energy after the split varies widely from run
to run, from thousands to about a million in ours; after the remeshing it is below the target.

## Notes

- **On files.** Each step is also a `wmtk_app` JSON file or a call `wm.<component>(options)`
  without meshes, reading and writing `.msh` files, which keep the regions. The envelope does not
  travel that way: the `.msh` simwild writes holds the mesh only.
- **Names.** The regions keep the names given in step 1 throughout, and every option selects them
  by name: `a & b` in step 2, `a` in step 3. The background is `_` in simwild's expressions and
  `ambient` in topological offset's (see [Regions](../../concepts/regions.md)).

## See also

- [SimWild](../simwild.md): every operation, and the envelope a run returns
- [Topological offset](../topological_offset.md): the band, its tags, and the optimization
- [Regions](../../concepts/regions.md): the `tags` attribute and the expression language
- [Python API](../../python-api.md#meshes-and-files): `wm.save` and `wm.load`
